```py
from PIL import Image
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader
import tempfile
import os

img_path = "./input.png"
pdf_path = "output.pdf"

img = Image.open(img_path).convert("RGB")
img_w, img_h = img.size

# A4 页面大小（单位 point）
page_w, page_h = A4

# 留一点边距
margin = 20
usable_w = page_w - 2 * margin
usable_h = page_h - 2 * margin

# 按 PDF 可用宽度计算缩放比例
scale = usable_w / img_w

# 反推：每页最多能放原图多高
slice_h = int(usable_h / scale)

c = canvas.Canvas(pdf_path, pagesize=A4)

y = 0
page_num = 1

while y < img_h:
    box = (0, y, img_w, min(y + slice_h, img_h))
    part = img.crop(box)

    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
        temp_path = tmp.name
        part.save(temp_path, format="PNG")

    part_w, part_h = part.size
    draw_w = usable_w
    draw_h = part_h * scale

    # 从页面上边开始放
    x_pos = margin
    y_pos = page_h - margin - draw_h

    c.drawImage(
        ImageReader(temp_path),
        x_pos,
        y_pos,
        width=draw_w,
        height=draw_h,
        preserveAspectRatio=True,
        mask='auto'
    )

    c.showPage()
    os.remove(temp_path)

    y += slice_h
    page_num += 1

c.save()
print(f"saved to {pdf_path}")

```