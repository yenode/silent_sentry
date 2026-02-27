#!/usr/bin/env python3
"""
VLM Costmap Node
================
Zero-shot Vision-Language Model traversability costmap layer.
Uses a CLIP-style model to score terrain traversability from camera images
without any fine-tuning on desert data.

Subscribes:
  /camera/image_raw   (sensor_msgs/Image)

Publishes:
  /terrain_class      (std_msgs/Int8)    — 0=open 1=dune 2=rocks
  /vlm_costmap/score  (std_msgs/Float32) — raw traversability score [0..1]

Classes used for zero-shot prompting:
  - "open sandy desert corridor, safe to drive"       → class 0
  - "sand dune slope, difficult terrain"              → class 1
  - "rocky ground, dangerous for vehicle"             → class 2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Float32
import numpy as np


class VLMCostmapNode(Node):
    def __init__(self):
        super().__init__('vlm_costmap_node')

        self.declare_parameter('model_name', 'openai/clip-vit-base-patch32')
        self.declare_parameter('inference_rate_hz', 2.0)
        self._model_name = self.get_parameter('model_name').value
        self._rate       = self.get_parameter('inference_rate_hz').value

        self._latest_image = None
        self._model        = None
        self._processor    = None

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self._img_cb, 10)

        # Publishers
        self._class_pub = self.create_publisher(Int8,    '/terrain_class',     10)
        self._score_pub = self.create_publisher(Float32, '/vlm_costmap/score', 10)

        # Inference timer
        self.create_timer(1.0 / self._rate, self._infer)

        self.get_logger().info(
            f'VLM Costmap ready | model={self._model_name} | '
            f'rate={self._rate} Hz (lazy-loaded on first image)'
        )

    # ── Lazy model load ───────────────────────────────────────────────────────
    def _load_model(self):
        try:
            from transformers import CLIPProcessor, CLIPModel
            import torch
            self._model     = CLIPModel.from_pretrained(self._model_name)
            self._processor = CLIPProcessor.from_pretrained(self._model_name)
            self.get_logger().info('CLIP model loaded successfully')
        except ImportError:
            self.get_logger().warn(
                'transformers not installed — running in STUB mode '
                '(random terrain class for development)'
            )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _img_cb(self, msg: Image):
        self._latest_image = msg

    # ── Inference ─────────────────────────────────────────────────────────────
    def _infer(self):
        if self._latest_image is None:
            return

        if self._model is None and self._processor is None:
            self._load_model()

        if self._model is not None:
            terrain_class, score = self._clip_infer(self._latest_image)
        else:
            # Stub: random class for development without GPU
            terrain_class = int(np.random.choice([0, 1, 2], p=[0.6, 0.3, 0.1]))
            score         = float(np.random.uniform(0.3, 1.0))

        self._class_pub.publish(Int8(data=terrain_class))
        self._score_pub.publish(Float32(data=score))

    def _clip_infer(self, img_msg: Image):
        """Run CLIP zero-shot classification."""
        import torch
        from PIL import Image as PILImage

        # Convert ROS Image → PIL
        img_array = np.frombuffer(img_msg.data, dtype=np.uint8)
        img_array = img_array.reshape(img_msg.height, img_msg.width, -1)
        pil_img   = PILImage.fromarray(img_array[..., :3])

        prompts = [
            'open sandy desert corridor, safe to drive through',
            'sand dune slope, difficult off-road terrain',
            'rocky ground with boulders, dangerous for vehicle',
        ]
        inputs = self._processor(
            text=prompts, images=pil_img, return_tensors='pt', padding=True
        )
        with torch.no_grad():
            logits = self._model(**inputs).logits_per_image
            probs  = logits.softmax(dim=1).squeeze().numpy()

        terrain_class = int(np.argmax(probs))
        score         = float(probs[0])  # probability of "safe" class
        return terrain_class, score


def main(args=None):
    rclpy.init(args=args)
    node = VLMCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
