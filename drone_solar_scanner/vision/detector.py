"""
Deep Learning Solar Panel Dust & Soiling Detection Engine.
Loads the trained ResNet-18 model weights (resnet18_solar_dust.pth) and performs
high-speed, real-time inference on solar panel imagery.
"""

import os
from dataclasses import dataclass
from typing import Union, Dict, Any, Optional
import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
import numpy as np


@dataclass
class DetectionResult:
    """Detection inference result for an inspected solar panel frame."""
    class_id: int             # 0 = Clean, 1 = Dusty
    label: str                # 'Clean' or 'Dusty'
    is_dusty: bool            # True if panel has dust/soiling
    confidence: float         # Softmax probability for predicted class [0.0, 1.0]
    clean_prob: float         # Probability of being clean
    dusty_prob: float         # Probability of being dusty

    def to_dict(self) -> Dict[str, Any]:
        return {
            "class_id": self.class_id,
            "label": self.label,
            "is_dusty": self.is_dusty,
            "confidence": round(self.confidence, 4),
            "clean_prob": round(self.clean_prob, 4),
            "dusty_prob": round(self.dusty_prob, 4)
        }


class SolarDustDetector:
    """
    ResNet-18 PyTorch classifier for solar panel contamination detection.
    """

    CLASSES = ["Clean", "Dusty"]

    def __init__(self, model_weights_path: str = "resnet18_solar_dust.pth", device: Optional[str] = None):
        self.weights_path = model_weights_path

        if device:
            self.device = torch.device(device)
        else:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])

        self.model = self._build_model()
        self._load_weights()

    def _build_model(self) -> nn.Module:
        """Instantiates ResNet18 architecture with the exact custom MLP classification head."""
        model = models.resnet18(weights=None)
        in_features = model.fc.in_features

        model.fc = nn.Sequential(
            nn.Linear(in_features, 50),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(50, 20),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(20, 10),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(10, 2)
        )
        return model

    def _load_weights(self):
        """Loads trained weights onto model and switches to eval mode."""
        if not os.path.exists(self.weights_path):
            raise FileNotFoundError(f"Model checkpoint not found at: {self.weights_path}")

        state_dict = torch.load(self.weights_path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(state_dict)
        self.model.to(self.device)
        self.model.eval()

    def predict(self, image_input: Union[str, Image.Image, np.ndarray]) -> DetectionResult:
        """
        Runs neural network inference on an image path, PIL Image, or numpy array.
        
        Args:
            image_input: File path, PIL Image, or OpenCV/numpy BGR/RGB array.
            
        Returns:
            DetectionResult with classification, confidence, and class probabilities.
        """
        if isinstance(image_input, str):
            img = Image.open(image_input).convert("RGB")
        elif isinstance(image_input, np.ndarray):
            # Check if OpenCV BGR (3 channels) and convert to RGB
            if len(image_input.shape) == 3 and image_input.shape[2] == 3:
                img = Image.fromarray(image_input[:, :, ::-1])
            else:
                img = Image.fromarray(image_input)
        elif isinstance(image_input, Image.Image):
            img = image_input.convert("RGB")
        else:
            raise TypeError(f"Unsupported image input type: {type(image_input)}")

        tensor = self.transform(img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            outputs = self.model(tensor)
            probabilities = torch.softmax(outputs, dim=1).squeeze(0)
            class_id = int(torch.argmax(probabilities).item())
            confidence = float(probabilities[class_id].item())
            clean_prob = float(probabilities[0].item())
            dusty_prob = float(probabilities[1].item())

        label = self.CLASSES[class_id]
        is_dusty = (class_id == 1)

        return DetectionResult(
            class_id=class_id,
            label=label,
            is_dusty=is_dusty,
            confidence=confidence,
            clean_prob=clean_prob,
            dusty_prob=dusty_prob
        )
