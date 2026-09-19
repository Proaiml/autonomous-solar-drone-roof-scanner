"""
Unit tests for PyTorch ResNet-18 Solar Panel Dust Detector.
Verifies model architecture, weights loading, and inference accuracy on sample data.
"""

import os
import pytest
from PIL import Image
import numpy as np

from drone_solar_scanner.vision.detector import SolarDustDetector, DetectionResult


MODEL_PATH = "resnet18_solar_dust.pth"
CLEAN_DIR = "tests/sample_data/clean"
DUSTY_DIR = "tests/sample_data/dusty"


def test_model_initialization():
    """Verifies that the detector initializes and loads weights successfully."""
    assert os.path.exists(MODEL_PATH), f"Model file {MODEL_PATH} must exist."
    detector = SolarDustDetector(model_weights_path=MODEL_PATH)
    assert detector.model is not None
    assert detector.device is not None


def test_clean_image_inference():
    """Verifies inference on real clean solar panel samples."""
    detector = SolarDustDetector(model_weights_path=MODEL_PATH)
    clean_files = [os.path.join(CLEAN_DIR, f) for f in os.listdir(CLEAN_DIR) if f.endswith(".jpg")]
    assert len(clean_files) > 0, "Clean sample files must exist."

    for img_path in clean_files:
        result = detector.predict(img_path)
        assert isinstance(result, DetectionResult)
        assert result.label in ["Clean", "Dusty"]
        assert 0.0 <= result.confidence <= 1.0
        assert pytest.approx(result.clean_prob + result.dusty_prob, rel=1e-3) == 1.0
        # Clean samples should have high clean probability
        assert result.clean_prob > 0.5
        assert result.label == "Clean"
        assert not result.is_dusty


def test_dusty_image_inference():
    """Verifies inference on real dusty solar panel samples."""
    detector = SolarDustDetector(model_weights_path=MODEL_PATH)
    dusty_files = [os.path.join(DUSTY_DIR, f) for f in os.listdir(DUSTY_DIR) if f.endswith(".jpg")]
    assert len(dusty_files) > 0, "Dusty sample files must exist."

    # Test at least one distinctly dusty sample
    tested_results = [detector.predict(img_path) for img_path in dusty_files]
    assert any(r.is_dusty for r in tested_results), "At least one dusty sample must be detected as Dusty."


def test_numpy_and_pil_inputs():
    """Verifies detector accepts PIL Images and NumPy arrays in addition to paths."""
    detector = SolarDustDetector(model_weights_path=MODEL_PATH)
    sample_path = os.path.join(CLEAN_DIR, os.listdir(CLEAN_DIR)[0])

    # 1. PIL Image
    pil_img = Image.open(sample_path)
    res_pil = detector.predict(pil_img)
    assert isinstance(res_pil, DetectionResult)

    # 2. NumPy array (e.g. from OpenCV)
    np_arr = np.array(pil_img)
    res_np = detector.predict(np_arr)
    assert isinstance(res_np, DetectionResult)
    assert res_pil.class_id == res_np.class_id
