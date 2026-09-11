#!/usr/bin/env python3
"""Gera o marcador ArUco do follow-me pronto para impressão (A4, tamanho real).

Uso: make_aruco.py [id] [lado_cm] [saida.png]
O lado é o do quadrado preto; imprima em A4 sem "ajustar à página" e confira com uma régua.
"""
import sys
import cv2
import numpy as np

marker_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
side_cm = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
out = sys.argv[3] if len(sys.argv) > 3 else 'aruco_id%d_%.0fcm.png' % (marker_id, side_cm)

DPI = 300
side_px = int(round(side_cm / 2.54 * DPI))
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# OpenCV >= 4.7 usa generateImageMarker; versões anteriores, drawMarker
if hasattr(cv2.aruco, 'generateImageMarker'):
    img = cv2.aruco.generateImageMarker(d, marker_id, side_px)
else:
    img = cv2.aruco.drawMarker(d, marker_id, side_px)

# borda branca (o detector precisa dela) + folha A4 em retrato
quiet = int(side_px * 0.15)
img = cv2.copyMakeBorder(img, quiet, quiet, quiet, quiet, cv2.BORDER_CONSTANT, value=255)
page = np.full((int(29.7 / 2.54 * DPI), int(21.0 / 2.54 * DPI)), 255, np.uint8)
y = (page.shape[0] - img.shape[0]) // 2
x = (page.shape[1] - img.shape[1]) // 2
if y < 0 or x < 0:
    raise SystemExit('marcador de %.0f cm não cabe em A4 com a borda; use até ~15 cm' % side_cm)
page[y:y + img.shape[0], x:x + img.shape[1]] = img
cv2.putText(page, 'ArUco DICT_4X4_50  id=%d  lado=%.0f cm  (imprimir em A4, escala 100%%)'
            % (marker_id, side_cm), (x, y + img.shape[0] + 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
cv2.imwrite(out, page)
print('gerado %s  (lado do quadrado preto: %.1f cm a %d dpi)' % (out, side_cm, DPI))
