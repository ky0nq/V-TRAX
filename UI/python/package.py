from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

from PIL import Image


base = Path(__file__).resolve().parent
frames = [Image.open(path).convert("RGB").resize((1080, 480))
          for path in sorted((base / "motion_frames").glob("*.png"))]
if frames:
    frames[0].save(base / "motion_preview.gif", save_all=True,
                   append_images=frames[1:], duration=130, loop=0)

files = list(base.glob("*.qml"))
files += [base / name for name in (
    "main.py", "requirements.txt", "README.md", "preview.png",
    "angle_preview.png", "motion_preview.gif")]
files += list((base / "assets").rglob("*"))
files += list((base / "tests").glob("*.py"))

archive = base.parent / f"{base.name}.zip"
with ZipFile(archive, "w", ZIP_DEFLATED) as zip_file:
    for path in files:
        if path.is_file():
            zip_file.write(path, f"{base.name}/{path.relative_to(base)}")
print(archive)
