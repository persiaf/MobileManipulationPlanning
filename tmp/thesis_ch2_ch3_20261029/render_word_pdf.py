"""Use the skill renderer with an already verified Word PDF as conversion backend."""
from pathlib import Path
import importlib.util, sys

ROOT=Path(__file__).resolve().parent
SKILL=Path(r'C:\Users\Administrator\.codex\plugins\cache\openai-primary-runtime\documents\26.1026.10807\skills\documents\render_docx.py')
spec=importlib.util.spec_from_file_location('document_skill_renderer',SKILL)
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
pdf=ROOT/'qa.pdf'
assert pdf.exists() and pdf.stat().st_size>1000
mod.convert_to_pdf=lambda *args,**kwargs:(str(pdf),'PDF exported by Microsoft Word 16.0')
paths=mod.rasterize(str(ROOT/'第二章与第三章初稿_20261029.docx'),str(ROOT/'render'),110,False,False)
print(paths)
