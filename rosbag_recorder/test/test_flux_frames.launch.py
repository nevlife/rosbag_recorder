"""Record Flux frames that carry aux bytes or fields in the publisher's arena, as flux_bag does."""

import ctypes
import gc
import importlib.util
import os
from pathlib import Path
import shutil
import struct
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import flux
from flux_msgs.msg import FluxFrame
import flux_wrapper
from launch import LaunchDescription
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag_recorder_msgs.msg import TopicSelection
from rosbag_recorder_msgs.srv import ListTopics
from rosbag_recorder_msgs.srv import SetTopics
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Stop
from sensor_msgs.msg import Image as RosImage
from sensor_msgs_flux import image_ros
from sensor_msgs_flux.image import Image

ROOT = Path(tempfile.mkdtemp(prefix='recorder_flux_frames_'))
FINGERPRINT = 0xACDC5678
NO_GPU = flux.probe().route == flux.Route.NONE


def device_fill(span, value, nbytes, stream):
    """Fill `nbytes` of the device memory that `span` names with `value`, on `stream`."""
    cuda = ctypes.CDLL('libcuda.so.1')
    ctx = ctypes.c_void_p()
    assert cuda.cuInit(0) == 0
    assert cuda.cuDevicePrimaryCtxRetain(ctypes.byref(ctx), 0) == 0
    assert cuda.cuCtxPushCurrent_v2(ctx) == 0
    ptr = span.__cuda_array_interface__['data'][0]
    rc = cuda.cuMemsetD8Async(
        ctypes.c_uint64(ptr), ctypes.c_ubyte(value), ctypes.c_size_t(nbytes),
        ctypes.c_void_p(stream),
    )
    cuda.cuCtxPopCurrent_v2(ctypes.byref(ctypes.c_void_p()))
    assert rc == 0, rc


def recorder(*args, **kwargs):
    """Build the container of the installed launch file, with its remaps."""
    path = Path(get_package_share_directory('rosbag_recorder')) / 'launch' / 'recorder.launch.py'
    spec = importlib.util.spec_from_file_location('recorder_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.recorder(*args, **kwargs)


@pytest.mark.launch_test
def generate_test_description():
    """Launch one recorder over an empty catalog."""
    (ROOT / 'profiles').mkdir()
    (ROOT / 'catalog.yaml').write_text('groups: {}\n')
    container = recorder(overrides={'config_dir': str(ROOT), 'output_dir': str(ROOT / 'bags')})
    return LaunchDescription([container, launch_testing.actions.ReadyToTest()])


def split_meta(meta):
    """(aux_size, aux) of a raw FluxFrame's meta: (0, b'') without an aux tail."""
    meta = bytes(meta)
    end = 10 + 8 * meta[9]
    if len(meta) == end:
        return 0, b''
    version, aux_size, aux_len = struct.unpack_from('<III', meta, end)
    assert version == 1 and len(meta) == end + 12 + aux_len
    return aux_size, meta[end + 12:]


class TestFluxFrames(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('recorder_flux_frames_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(ROOT)

    def call(self, kind, name, **fields):
        client = self.node.create_client(kind, '/rosbag_recorder/' + name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(kind.Request(**fields))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
            self.assertTrue(future.done())
            return future.result()
        finally:
            self.node.destroy_client(client)

    def until(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('condition did not become true')

    def record(self, name, publish):
        """Select the Flux channel `name`, run publish(), and return its frames from the bag."""
        self.until(
            lambda: any(
                t.name == name and t.transport == 'flux'
                for t in self.call(ListTopics, 'topics').topics
            )
        )
        selected = [TopicSelection(name=name, transport='flux')]
        self.assertTrue(self.call(SetTopics, 'set_topics', topics=selected).success)
        self.assertTrue(self.call(Start, 'start').success)
        time.sleep(0.5)
        sent = publish()
        time.sleep(0.5)
        stopped = self.call(Stop, 'stop')
        self.assertTrue(stopped.success, stopped.message)
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=stopped.bag_path),
            rosbag2_py.ConverterOptions('cdr', 'cdr'),
        )
        frames = []
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == name + '/flux':
                frames.append(deserialize_message(data, FluxFrame))
        return sent, frames

    def test_aux_bytes_and_the_aux_size_are_recorded(self):
        pub = flux.Publisher(
            self.node, '/frames/aux', fingerprint=FINGERPRINT, slot_size=4096, slot_count=4,
            aux_size=32,
        )

        def publish():
            sent = []
            for i in range(20):
                aux = bytes([i + 1]) * (i % 33)
                loan = pub.loan((4,), 'u1')
                loan.array[:] = i
                loan.aux[:len(aux)] = np.frombuffer(aux, np.uint8)
                loan.aux_len = len(aux)
                self.assertEqual(loan.commit(), flux.Published.OK)
                sent.append((bytes([i]) * 4, aux))
                time.sleep(0.02)
            return sent

        try:
            sent, frames = self.record('/frames/aux', publish)
        finally:
            pub = None
            gc.collect()
        self.assertTrue(frames)
        recorded = [(bytes(f.data), split_meta(f.meta)) for f in frames]
        self.assertEqual({aux_size for _, (aux_size, _) in recorded}, {32})
        self.assertEqual([(d, aux) for d, (_, aux) in recorded], sent[len(sent) - len(frames):])

    def test_fields_in_the_arena_are_recorded_in_the_frame(self):
        pub = flux.Publisher(
            self.node, '/frames/arena', fingerprint=Image.FINGERPRINT__, slot_size=4096,
            slot_count=4,
        )
        sizes = (16, 8192, 16, 8192)

        def publish():
            for i, n in enumerate(sizes):
                b = Image.build__(pub)
                b.height, b.width, b.encoding, b.step = 1, n, 'mono8', n
                b.alloc__data(n)[:] = i
                self.assertEqual(b.commit__(), flux.Published.OK)
                time.sleep(0.05)
            return sizes

        try:
            _, frames = self.record('/frames/arena', publish)
        finally:
            pub = None
            gc.collect()
        # The bag's copy holds the bytes that were in the arena: the frame is long enough for them,
        # and its descriptor of data points inside it.
        self.assertEqual(len(frames), len(sizes))
        self.assertTrue(all(len(bytes(f.data)) >= n for f, n in zip(frames, sizes)))
        got = []
        for f in frames:
            img = Image.View(np.frombuffer(bytes(f.data), np.uint8))
            data = np.asarray(img.data)
            got.append((img.width, len(data), int(data[-1])))
        self.assertEqual(got, [(n, n, i) for i, n in enumerate(sizes)])

    # The channel of a flux_wrapper Image publisher with Device::Cuda: on a discrete GPU its slots
    # are device memory, and the recorder's CPU subscription takes a host copy of each frame.
    @unittest.skipIf(NO_GPU, 'needs a CUDA device')
    def test_a_gpu_channel_with_aux_bytes_is_recorded(self):
        n = 64 * 64
        pub = flux.Publisher(
            self.node, '/frames/gpu', fingerprint=flux.gpu_fingerprint(Image.FINGERPRINT__),
            slot_size=1 << 16, slot_count=4, device='cuda', aux_size=4096,
        )
        self.assertTrue(pub.device_backed or flux.probe().integrated)

        def publish():
            sent = []
            for i in range(20):
                aux = bytes([i + 1]) * (i + 1)
                loan = pub.loan((n,), 'u1')
                device_fill(loan, i, n, loan.stream)
                loan.aux[:len(aux)] = np.frombuffer(aux, np.uint8)
                loan.aux_len = len(aux)
                self.assertEqual(loan.commit(), flux.Published.OK)
                sent.append((bytes([i]) * n, aux))
                time.sleep(0.02)
            return sent

        try:
            sent, frames = self.record('/frames/gpu', publish)
        finally:
            pub = None
            gc.collect()
        self.assertGreater(len(frames), 10)
        recorded = [(bytes(f.data), split_meta(f.meta)) for f in frames]
        self.assertEqual({aux_size for _, (aux_size, _) in recorded}, {4096})
        self.assertEqual([(d, aux) for d, (_, aux) in recorded], sent[len(sent) - len(frames):])

    @unittest.skipIf(NO_GPU, 'needs a CUDA device')
    def test_fields_in_the_gpu_arena_are_recorded_in_the_frame(self):
        pub = flux.Publisher(
            self.node, '/frames/gpu_arena', fingerprint=Image.FINGERPRINT__, device='cuda',
            slot_size=4096, slot_count=4,
        )
        values = (5, 6, 7)

        def publish():
            for v in values:
                loan = pub.loan(dtype='uint8')
                b = Image.Builder(loan.array, loan)
                b.height, b.width, b.encoding, b.step = 1, 8192, 'mono8', 8192
                device_fill(b.alloc_device__data(8192), v, 8192, loan.stream)
                self.assertEqual(b.commit__(), flux.Published.OK)
                time.sleep(0.05)
            return values

        try:
            _, frames = self.record('/frames/gpu_arena', publish)
        finally:
            pub = None
            gc.collect()
        got = []
        for f in frames:
            data = np.asarray(Image.View(np.frombuffer(bytes(f.data), np.uint8)).data)
            got.append((len(data), int(data[0]), int(data[-1])))
        self.assertEqual(got, [(8192, v, v) for v in values])

    # An application's flux_wrapper Image publisher with device="cuda": the fields but data come
    # back from the aux bytes and the data from the host copy.
    @unittest.skipIf(NO_GPU, 'needs a CUDA device')
    def test_a_flux_wrapper_cuda_image_is_recorded(self):
        os.environ['ENABLE_FLUX'] = '1'
        pub = flux_wrapper.Publisher(
            self.node, RosImage, '/frames/wrapper', 10, device='cuda', slot_count=4
        )
        self.assertTrue(pub.uses_device)

        def publish():
            sent = []
            for i in range(20):
                m = RosImage(height=4, width=8, encoding='rgb8', step=24)
                m.header.stamp.sec, m.header.frame_id = i + 1, f'camera_{i}'
                m.data = bytes(range(i, i + 96))
                self.assertTrue(pub.publish(m))
                sent.append(m)
                time.sleep(0.02)
            return sent

        try:
            sent, frames = self.record('/frames/wrapper', publish)
        finally:
            pub = None
            gc.collect()
        self.assertGreater(len(frames), 10)
        recorded = []
        for f in frames:
            m = image_ros.frame_to_msg(Image.View(split_meta(f.meta)[1]))
            m.data = bytes(f.data)
            recorded.append(m)
        self.assertEqual(recorded, sent[len(sent) - len(frames):])
