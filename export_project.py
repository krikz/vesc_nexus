#!/usr/bin/env python3

import os
import sys
from pathlib import Path

# -----------------------------
# Настройки
# -----------------------------

# Корневая директория проекта (можно передать как аргумент)
ROOT_DIR = sys.argv[1] if len(sys.argv) > 1 else "."

# Имя выходного файла
OUTPUT_FILE = "project_export.txt"

# Расширения файлов, которые нужно включить
INCLUDE_EXTENSIONS = {
    '.cpp', '.cxx', '.cc',
    '.h', '.hpp', '.hxx', '.hh',
    '.c', '.h',
    '.py',
    '.yaml', '.yml',
    '.xml', '.xacro',
    '.msg',
    '.launch', '.py', '.yaml',
    '.txt',
    '.md',
    '.dockerfile', 'dockerfile',
    '.cmake', 'cmakelists.txt',
    '.gitignore',
    '.json',
    '.sh', '.bash'
}

# Папки, которые нужно пропустить
EXCLUDE_DIRS = {
    'build', 'install', 'log', '__pycache__',
    '.git', '.vscode', '.idea', 'venv', 'env',
    'devel', 'dist', 'node_modules',
    '__pycache__', '.pytest_cache', '.mypy_cache'
}

# Файлы, которые нужно пропустить
EXCLUDE_FILES = {
    'export_project.py',  # Будет включён, но если хочешь исключить — добавь сюда
}

# -----------------------------
# Скрипт
# -----------------------------

def should_include_file(path):
    if path.name.lower() in EXCLUDE_FILES:
        return False
    return path.suffix.lower() in INCLUDE_EXTENSIONS or path.name.lower() in ['cmakelists.txt', 'dockerfile']

def should_include_dir(dir_name):
    return dir_name not in EXCLUDE_DIRS

def read_file_safely(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        return content
    except Exception as e:
        return f"<Ошибка чтения файла: {e}>"

def export_project(root_dir, output_file):
    root_path = Path(root_dir).resolve()
    if not root_path.exists():
        print(f"❌ Указанный путь не существует: {root_path}")
        return

    with open(output_file, 'w', encoding='utf-8') as out:
        # Запишем информацию о проекте
        out.write(f"# Экспорт проекта: {root_path.name}\n")
        out.write(f"# Сгенерировано: export_project.py\n")
        out.write(f"# Время: {__import__('datetime').datetime.now().isoformat()}\n")
        out.write(f"# Путь: {root_path}\n")
        out.write(f"{'='*80}\n\n")

        file_count = 0
        for dirpath, dirnames, filenames in os.walk(root_path):
            dirnames[:] = [d for d in dirnames if should_include_dir(d)]  # Фильтруем папки
            dirpath_obj = Path(dirpath)

            # Пропускаем, если это исключённая директория
            if not should_include_dir(dirpath_obj.name):
                continue

            rel_dir = dirpath_obj.relative_to(root_path)

            for filename in sorted(filenames):
                file_path = Path(dirpath) / filename
                if not should_include_file(file_path):
                    continue

                rel_file_path = file_path.relative_to(root_path)
                content = read_file_safely(file_path)

                out.write(f"# {'-'*76}\n")
                out.write(f"# Файл: {rel_file_path}\n")
                out.write(f"# Путь: {file_path}\n")
                out.write(f"# {'-'*76}\n")
                out.write(content)
                out.write(f"\n\n")

                file_count += 1

        out.write(f"# {'='*80}\n")
        out.write(f"# Экспорт завершён. Включено файлов: {file_count}\n")

    print(f"✅ Проект экспортирован в: {output_file}")
    print(f"📁 Обработано: {root_path}")
    print(f"📄 Файлов включено: {file_count}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("📌 Использование:")
        print("  python export_project.py <путь_к_проекту>")
        print("  Пример:")
        print("  python export_project.py .")
        print("  python export_project.py ~/ros2_ws/src")
        sys.exit(1)

    export_project(ROOT_DIR, OUTPUT_FILE)