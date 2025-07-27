#!/usr/bin/env python3

# Este es un enlace al script principal
# El script real está en braccio_vision/vision_demo.py

import os
import sys

# Añadir el directorio del paquete al path
package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, package_dir)

from braccio_vision.vision_demo import main

if __name__ == '__main__':
    main()
