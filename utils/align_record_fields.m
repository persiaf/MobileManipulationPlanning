function aligned = align_record_fields(record, template)
%ALIGN_RECORD_FIELDS 将任意 record 对齐到统一模板字段。
%
%   aligned = align_record_fields(record, template)
%
%   输出 aligned 只包含 template 中已有字段，字段顺序也与 template 一致。
%   record 中缺失的字段使用 template 默认值；record 中多余字段会被忽略。

    aligned = template;
    names = fieldnames(template);

    for i = 1:numel(names)
        fn = names{i};
        if isstruct(record) && isfield(record, fn)
            aligned.(fn) = record.(fn);
        end
    end
end
