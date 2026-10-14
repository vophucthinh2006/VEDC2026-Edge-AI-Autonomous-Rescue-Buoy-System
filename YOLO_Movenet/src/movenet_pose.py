import os
import sys
import cv2
import numpy as np

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

class MoveNetPose:
    def __init__(self, model_path):
        if not model_path or not os.path.exists(model_path):
            raise FileNotFoundError(f"Không tìm thấy model MoveNet TFLite tại: {model_path}")
        
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def estimate_pose(self, crop_rgb_192):
        detail_in = self.input_details[0]
        dtype = detail_in['dtype']

        if dtype == np.int8:
            input_data = (crop_rgb_192.astype(np.int32) - 128).astype(np.int8)
            input_data = np.expand_dims(input_data, axis=0)
        elif dtype == np.uint8:
            input_data = np.expand_dims(crop_rgb_192.astype(np.uint8), axis=0)
        else:
            input_data = np.expand_dims(crop_rgb_192.astype(np.float32), axis=0)

        self.interpreter.set_tensor(detail_in['index'], input_data)
        self.interpreter.invoke()

        detail_out = self.output_details[0]
        raw_out = self.interpreter.get_tensor(detail_out['index'])

        if detail_out['dtype'] == np.int8 or detail_out['dtype'] == np.uint8:
            scale, zero_point = detail_out['quantization']
            if scale > 0:
                raw_out = (raw_out.astype(np.float32) - zero_point) * scale

        return raw_out.reshape(17, 3) # Mảng 2D [17, 3] -> (y, x, score)