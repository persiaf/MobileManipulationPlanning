"""Structural and provenance checks for the standalone revision."""
from pathlib import Path
from docx import Document
import json, re, hashlib

ROOT = Path(__file__).resolve().parent
DOC = ROOT / '第二章与第三章修订稿_20261029.docx'
blocks = json.loads((ROOT / 'final_content.json').read_text(encoding='utf-8'))
d = Document(DOC)
labels = [b['label'] for b in blocks if b['type'] == 'eq']
actual = [m.group(1) for p in d.paragraphs if (m := re.fullmatch(r'\s*（([23]-\d+)）', p.text))]
assert labels == actual
assert len(set(labels)) == len(labels)
assert len(d.inline_shapes) == 3 and len(d.tables) == 9
for ch, count in [(2, 53), (3, 50)]:
    assert [x for x in labels if x.startswith(str(ch))] == [f'{ch}-{i}' for i in range(1, count + 1)]
text = '\n'.join(b.get('text', '') for b in blocks)
assert all(x in labels for x in re.findall(r'[（(]([23]-\d+)[）)]', text))
assert not re.search(r'表[23]-A\d+|[（(][23]-A\d+[）)]', text)
assert not (ROOT / 'pandoc.log').read_text(encoding='utf-8').strip()
old = json.loads((ROOT / 'baseline_content.json').read_text(encoding='utf-8-sig'))
for number in ['3-2', '3-3', '3-4']:
    t = next(b for b in old if b['type'] == 'table' and b['caption'].startswith('表' + number + ' '))
    assert any(t['rows'] == b['rows'] and t['headers'] == b['headers'] for b in blocks if b['type'] == 'table')
sources = {
    r'D:\file_used\西柚\实验室_任务\硕士论文相关\大论文\初稿.docx': '4098583f110200262cbf1aa0736952b7f257b0d209e6f57afa1a62e8bf88e503',
    r'D:\file_used\西柚\实验室_任务\硕士论文相关\大论文\chatgpt生成文件\第二章_MathType_纯公式代码.txt': 'b8376ce6499e9c907e61c6a4e6aa42f2c6bd73343a4f5f8e81a5cab4dbbd39df',
    r'D:\file_used\西柚\实验室_任务\硕士论文相关\大论文\chatgpt生成文件\第二章与第三章初稿_20261029.docx': '1d7168f2afabb24af48ffc32c55cc175997f9634ce3226eb0a74c26e92dbdd76'
}
for path, expected in sources.items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
stats = {
    'equations': len(labels), 'tables': len(d.tables), 'figures': len(d.inline_shapes),
    'native_math_nodes': len(d.element.xpath('//m:oMath')),
    'pages': len((ROOT / 'qa_layout.txt').read_text(encoding='utf-8').split('\f')) - 1,
    'equation_references_valid': True, 'pandoc_warnings': False,
    'historical_experimental_tables_unchanged': True, 'original_files_unchanged': True,
    'sha256': hashlib.sha256(DOC.read_bytes()).hexdigest(), 'output': str(DOC)
}
(ROOT / 'final_quality_check.json').write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(stats, ensure_ascii=False, indent=2))
