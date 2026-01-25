# 代码引用与源代码链接

## 概述

MkDocs 的 `pymdownx.snippets` 扩展本身**不支持**直接点击代码跳转到源文件，但我们可以通过以下方式实现类似功能：

1. ✅ **添加 GitHub 链接按钮**（推荐）
2. ✅ **使用代码标题显示文件路径**
3. ⚠️ **使用代码注释添加链接**

## 方法 1：添加 GitHub 链接按钮（推荐）

### 配置

在 `mkdocs.yml` 中启用必要的功能：

```yaml
theme:
  features:
    - content.code.annotate # 启用代码注释
    
# Repository 信息
repo_name: yangtzech/MotionPlanning
repo_url: https://github.com/yangtzech/MotionPlanning
```

### 使用示例

在代码块**上方**添加链接按钮：

```markdown
[:material-file-code: 查看源代码](https://github.com/yangtzech/MotionPlanning/blob/master/src/pyauto/control/Stanley.py#L21-L41){ .md-button }

\`\`\`python title="src/pyauto/control/Stanley.py"
--8<-- "src/pyauto/control/Stanley.py:compute_control_command"
\`\`\`
```

**效果**：
- 在代码块上方显示一个按钮
- 点击按钮跳转到 GitHub 对应代码位置
- 支持精确到行号

### GitHub 链接格式

| 目标 | URL 格式 | 示例 |
|------|---------|------|
| 完整文件 | `blob/master/path/file.py` | `blob/master/src/pyauto/control/Stanley.py` |
| 特定行 | `blob/master/path/file.py#L行号` | `blob/master/src/pyauto/control/Stanley.py#L21` |
| 行范围 | `blob/master/path/file.py#L开始-L结束` | `blob/master/src/pyauto/control/Stanley.py#L21-L41` |

### 获取行号

使用 `grep` 命令查找代码标记的行号：

```bash
# 查找开始行
grep -n "start:compute_control_command" src/pyauto/control/Stanley.py

# 查找结束行  
grep -n "end:compute_control_command" src/pyauto/control/Stanley.py
```

## 方法 2：使用代码标题

代码块的 `title` 参数会显示文件路径，帮助用户识别源文件位置：

```markdown
\`\`\`python title="src/pyauto/control/Stanley.py"
--8<-- "src/pyauto/control/Stanley.py:compute_control_command"
\`\`\`
```

**效果**：代码块顶部显示文件路径

## 方法 3：内联链接

在段落中添加指向源文件的链接：

```markdown
完整的实现请参考 [Stanley.py](https://github.com/yangtzech/MotionPlanning/blob/master/src/pyauto/control/Stanley.py#L21-L41)：

\`\`\`python
--8<-- "src/pyauto/control/Stanley.py:compute_control_command"
\`\`\`
```

## 完整示例

以下是 Stanley 控制器文档的完整示例：

````markdown
## 代码实现

### 核心代码

完整的 `ComputeControlCommand` 方法实现：

[:material-file-code: 查看源代码](https://github.com/yangtzech/MotionPlanning/blob/master/src/pyauto/control/Stanley.py#L21-L41){ .md-button }

```python title="src/pyauto/control/Stanley.py"
--8<-- "src/pyauto/control/Stanley.py:compute_control_command"
```

### 前轴中心计算

[:octicons-file-code-16: 源代码](https://github.com/yangtzech/MotionPlanning/blob/master/src/pyauto/control/Stanley.py#L24-L27){ .md-button .md-button--primary }

```python title="前轴中心计算"
--8<-- "src/pyauto/control/Stanley.py:front_axle_center"
```
````

## 可用图标

Material for MkDocs 支持以下图标库：

### Material Design Icons

```markdown
:material-file-code:     # 代码文件
:material-github:        # GitHub
:material-source-branch: # 分支
:material-file-document: # 文档
```

### Octicons (GitHub)

```markdown
:octicons-file-code-16:  # 代码文件
:octicons-mark-github-16: # GitHub 标记
:octicons-repo-16:       # 仓库
```

### FontAwesome

```markdown
:fontawesome-brands-github: # GitHub 品牌图标
:fontawesome-solid-code:    # 代码图标
```

完整图标列表：[Material Icons](https://squidfunk.github.io/mkdocs-material/reference/icons-emojis/)

## 按钮样式

### 基本按钮

```markdown
[链接文本](URL){ .md-button }
```

### 主要按钮（高亮）

```markdown
[链接文本](URL){ .md-button .md-button--primary }
```

### 带图标的按钮

```markdown
[:material-github: GitHub](URL){ .md-button }
```

## 自动化脚本

创建一个脚本来自动生成带行号的 GitHub 链接：

```python
#!/usr/bin/env python3
"""生成源代码 GitHub 链接"""

import re
import sys

def find_snippet_lines(file_path, snippet_name):
    """查找代码片段的行号"""
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    start_pattern = f"--8<-- \\[start:{snippet_name}\\]"
    end_pattern = f"--8<-- \\[end:{snippet_name}\\]"
    
    start_line = None
    end_line = None
    
    for i, line in enumerate(lines, 1):
        if re.search(start_pattern, line):
            start_line = i + 1  # 下一行是实际代码
        elif re.search(end_pattern, line):
            end_line = i - 1  # 上一行是实际代码
            break
    
    return start_line, end_line

def generate_github_link(repo_url, file_path, start_line, end_line):
    """生成 GitHub 链接"""
    base_url = repo_url.rstrip('/')
    if end_line and end_line > start_line:
        return f"{base_url}/blob/master/{file_path}#L{start_line}-L{end_line}"
    else:
        return f"{base_url}/blob/master/{file_path}#L{start_line}"

# 使用示例
if __name__ == "__main__":
    file_path = "src/pyauto/control/Stanley.py"
    snippet_name = "compute_control_command"
    repo_url = "https://github.com/yangtzech/MotionPlanning"
    
    start, end = find_snippet_lines(file_path, snippet_name)
    if start:
        link = generate_github_link(repo_url, file_path, start, end)
        print(f"[:material-file-code: 查看源代码]({link}){{ .md-button }}")
```

## 最佳实践

1. **统一风格**：在整个项目中使用相同的按钮样式和图标
2. **精确行号**：尽量提供准确的行号范围，方便读者快速定位
3. **保持同步**：当代码位置变化时，记得更新链接
4. **多种方式**：结合按钮、标题和内联链接，提供多种访问方式
5. **本地开发**：对于本地开发，可以考虑添加相对路径链接

## 限制

⚠️ **注意事项**：

- Snippet 引用的代码**无法**直接点击跳转
- 需要**手动维护** GitHub 链接和行号
- 代码位置变化时需要**更新链接**
- 仅适用于**公开仓库**（私有仓库需要认证）

## 替代方案

如果需要更强大的代码浏览功能，可以考虑：

1. **集成 API 文档**：使用 mkdocstrings 自动生成 API 文档
2. **代码高亮**：使用 CodeHilite 提供行号和高亮
3. **Git 集成**：使用 git-committers 显示代码贡献者
4. **搜索功能**：利用 MkDocs 的搜索找到相关代码

## 参考资料

- [MkDocs Material - Buttons](https://squidfunk.github.io/mkdocs-material/reference/buttons/)
- [MkDocs Material - Code Blocks](https://squidfunk.github.io/mkdocs-material/reference/code-blocks/)
- [PyMdown Extensions - Snippets](https://facelessuser.github.io/pymdown-extensions/extensions/snippets/)
- [Material Icons](https://squidfunk.github.io/mkdocs-material/reference/icons-emojis/)
