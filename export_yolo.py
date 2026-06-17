from ultralytics import YOLO

print("Downloading YOLOv8n model...")
model = YOLO('yolov8n.pt')

print("Exporting model to ONNX format...")
# Export the model to ONNX format. This is critical for Raspberry Pi CPU performance.
# We set imgsz=320 to improve framerate on the Pi 5.
success = model.export(format='onnx', imgsz=320, dynamic=False, simplify=True)

if success:
    print("\nExport successful! You can now find 'yolov8n.onnx' in the current directory.")
else:
    print("\nExport failed. Make sure you have the 'onnx' and 'onnxruntime' packages installed.")
    print("Run: pip install onnx onnxruntime")
