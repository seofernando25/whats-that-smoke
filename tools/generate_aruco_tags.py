from __future__ import annotations

import argparse
from pathlib import Path

import cv2
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import mm
from reportlab.pdfgen.canvas import Canvas


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--png-dir", type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.png_dir:
        args.png_dir.mkdir(parents=True, exist_ok=True)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    pdf = Canvas(str(args.output), pagesize=letter)
    page_w, page_h = letter
    pdf.setTitle("What's That Smoke - 50 mm ArUco Tags 0-4")
    pdf.setFont("Helvetica-Bold", 14)
    pdf.drawString(16 * mm, page_h - 16 * mm, "ARUCO DICT_4X4_50 - IDs 0-4")
    pdf.setFont("Helvetica", 8)
    pdf.drawString(16 * mm, page_h - 22 * mm, "Print at 100% / Actual Size. Black marker boundary = exactly 50 x 50 mm.")

    pdf.setLineWidth(0.4)
    pdf.line(16 * mm, page_h - 31 * mm, 116 * mm, page_h - 31 * mm)
    pdf.setFont("Helvetica", 7)
    pdf.drawString(118 * mm, page_h - 32 * mm, "100 mm check")

    positions = [(20, 175), (105, 175), (20, 90), (105, 90), (62.5, 10)]
    for tag_id, (x_mm, y_mm) in enumerate(positions):
        image = cv2.aruco.generateImageMarker(dictionary, tag_id, 1000, borderBits=1)
        png = (args.png_dir or args.output.parent) / f"aruco-4x4-50-id-{tag_id}.png"
        cv2.imwrite(str(png), image)
        # 5 mm white quiet zone on every side; marker's black outer boundary remains 50 mm.
        pdf.setFillColorRGB(1, 1, 1)
        pdf.rect((x_mm - 5) * mm, (y_mm - 5) * mm, 60 * mm, 66 * mm, fill=1, stroke=0)
        pdf.drawImage(str(png), x_mm * mm, y_mm * mm, 50 * mm, 50 * mm, preserveAspectRatio=True, mask="auto")
        pdf.setFillColorRGB(0, 0, 0)
        pdf.setFont("Helvetica-Bold", 10)
        pdf.drawCentredString((x_mm + 25) * mm, (y_mm - 3) * mm, f"ID {tag_id}")

    pdf.save()


if __name__ == "__main__":
    main()
