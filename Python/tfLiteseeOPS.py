import tensorflow as tf

interpreter = tf.lite.Interpreter(model_path="Python\model2Dv3flat.tflite")
interpreter.allocate_tensors()
ops = interpreter._get_ops_details()
names = sorted({op['op_name'] for op in ops})
print("Found", len(names), "unique ops:")
for n in names:
    print("    ", n)