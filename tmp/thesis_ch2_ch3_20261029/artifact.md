# Authoring contract

The user's request authorizes a new Word draft containing chapters 2 and 3, grounded in the outline in 初稿.docx and current MATLAB implementation. Original documents and code are read-only. The separately found historical outline is contextual evidence, not an instruction to change code or add unsupported functions. Current robot_config governs numerical parameters when historical materials conflict.

Reference: D:/file_used/西柚/实验室_任务/硕士论文相关/大论文/初稿.docx. Input fingerprints are recorded in source_hashes.json. This is a chapter-content completion, not recreation of the whole seven-chapter document.

Preserve the reference's A4 portrait geometry (21.00086 × 29.70036 cm), top/bottom 2.54 cm, left/right 3.175 cm; body Songti 12 pt with Times New Roman Latin, first-line 24 pt and 1.25 line spacing; black Hei chapter/section headings, 16/14/12 pt. Reuse source styles in a new reference copy. Clear inherited character-based indentation only to remove ambiguous overrides. Keep source unchanged. The new document has centered page numbers and chapters begin on new pages. No institutional formatting requirement has been supplied.

Required structure: 2.1–2.6 from initial draft, optional 2.7 chapter conclusion; 3.1–3.8 exactly retained. Add substantive third-level sections. No standalone cover, TOC, or additional chapter. Display equations are editable OMML; source MathType text is reference input and not a requested output format. Rebuild sequential chapter-specific numbering and verify cross-references. Tables have neutral thin gray rules and repeat header rows.

Grounding gates: verify current geometry, grid-coordinate convention, hard collision constraints versus soft costs, nominal versus optional robust mode, geometry proxy versus full IK, independent tasks versus simultaneous bimanual constraints. Real experiments must be tied to source CSVs and match current core code. Distinguish coverage from navigation success and finite-search failure from physical impossibility.

Render gate: Word COM is available outside the restricted login session. Export the new document with Word, rasterize its PDF using the document skill's rendering pipeline, inspect every page and fix overflow. Original Word files are not opened for writing. QA intermediates remain in this task-local folder; only the final DOCX is copied to the requested output directory.
