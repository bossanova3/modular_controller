import tensorflow as tf
import tf2onnx

# Carga tu modelo
model = tf.keras.models.load_model("mi_modelo_rnn.h5")

# Exportarlo
spec = (tf.TensorSpec((None, 1, 3), tf.float32, name="input"),)
output_path = "modelo_rnn.onnx"
model_proto, _ = tf2onnx.convert.from_keras(model, input_signature=spec, output_path=output_path)

print(f"Modelo guardado en {output_path}")