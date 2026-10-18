import os
import cv2
import numpy as np

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite as tflite
    except ImportError:
        tflite = None

try:
    import onnxruntime as ort
except ImportError:
    ort = None


class YOLOv8Detector:
    """Module YOLOv8 hỗ trợ tự động TFLite INT8 / FP32 và ONNX"""
    def __init__(self, model_path, conf_thresh=0.35, nms_thresh=0.45):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Không tìm thấy file model tại: {model_path}")
        
        self.model_path = model_path
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh
        self.is_tflite = model_path.endswith(".tflite")

        if self.is_tflite:
            if tflite is None:
                raise ImportError("Vui lòng cài tflite-runtime hoặc tensorflow!")
            self.interpreter = tflite.Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
        else:
            if ort is not None:
                self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
                self.input_name = self.session.get_inputs()[0].name
            else:
                self.net = cv2.dnn.readNetFromONNX(model_path)

    def detect(self, img_rgb, orig_shape, pad_info, ratio):
        if self.is_tflite:
            outputs = self._infer_tflite(img_rgb)
        else:
            outputs = self._infer_onnx(img_rgb)

        # Lấy kích thước thực tế của ảnh đầu vào (320 hoặc 640)
        in_h, in_w = img_rgb.shape[:2]
        return self._process_output(outputs, orig_shape, pad_info, ratio, input_size=(in_w, in_h))

    def _infer_tflite(self, img_rgb):
        detail_in = self.input_details[0]
        in_shape = detail_in['shape']
        dtype = detail_in['dtype']

        if len(in_shape) == 4 and in_shape[1] == 3:
            img_data = np.transpose(img_rgb, (2, 0, 1))  # NCHW
        else:
            img_data = img_rgb  # NHWC (Chuẩn TFLite)

        img_float = img_data.astype(np.float32) / 255.0

        if dtype in (np.int8, np.uint8):
            scale, zero_point = detail_in['quantization']
            if scale > 0:
                quant_data = (img_float / scale) + zero_point
                input_tensor = np.clip(quant_data, -128, 127).astype(dtype)
            else:
                input_tensor = (img_float * 255.0 - 128.0).astype(dtype)
        else:
            input_tensor = img_float

        if len(input_tensor.shape) == 3:
            input_tensor = np.expand_dims(input_tensor, axis=0)

        self.interpreter.set_tensor(detail_in['index'], input_tensor)
        self.interpreter.invoke()

        detail_out = self.output_details[0]
        raw_out = self.interpreter.get_tensor(detail_out['index'])

        if detail_out['dtype'] in (np.int8, np.uint8):
            scale, zero_point = detail_out['quantization']
            if scale > 0:
                raw_out = (raw_out.astype(np.float32) - zero_point) * scale

        return raw_out

    def _infer_onnx(self, img_rgb):
        blob = np.transpose(img_rgb, (2, 0, 1)).astype(np.float32) / 255.0
        blob = np.expand_dims(blob, axis=0)

        if hasattr(self, 'session'):
            return self.session.run(None, {self.input_name: blob})[0]
        else:
            self.net.setInput(blob)
            return self.net.forward()

    def _process_output(self, output, orig_shape, pad_info, ratio, input_size=(640, 640)):
        predictions = np.squeeze(output)
        
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        boxes, confidences, class_ids = [], [], []
        pad_w, pad_h = pad_info
        num_classes = predictions.shape[1] - 4
        
        in_w, in_h = input_size

        for pred in predictions:
            scores = pred[4:]
            class_id = 0 if num_classes == 1 else np.argmax(scores)
            confidence = float(scores[class_id])

            if confidence > self.conf_thresh:
                cx, cy, w, h = pred[0:4]

                # Quy đổi dựa trên kích thước thực tế truyền vào (in_w, in_h)
                if cx <= 1.0 and cy <= 1.0 and w <= 1.0 and h <= 1.0:
                    cx *= float(in_w)
                    cy *= float(in_h)
                    w *= float(in_w)
                    h *= float(in_h)

                x1 = int((cx - w / 2.0 - pad_w) / ratio)
                y1 = int((cy - h / 2.0 - pad_h) / ratio)
                bw = int(w / ratio)
                bh = int(h / ratio)

                x1 = max(0, x1)
                y1 = max(0, y1)
                bw = min(orig_shape[1] - x1, bw)
                bh = min(orig_shape[0] - y1, bh)

                if bw > 8 and bh > 8:
                    boxes.append([x1, y1, bw, bh])
                    confidences.append(confidence)
                    class_ids.append(int(class_id))

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_thresh, self.nms_thresh)
        
        final_detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                final_detections.append((boxes[i], confidences[i], class_ids[i]))
        return final_detections