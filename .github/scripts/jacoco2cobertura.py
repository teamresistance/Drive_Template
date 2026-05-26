#!/usr/bin/env python3
"""Convert JaCoCo XML report to Cobertura XML format."""

import sys
import xml.etree.ElementTree as ET


def convert(jacoco_path, source_root):
    root = ET.parse(jacoco_path).getroot()

    cobertura = ET.Element("coverage")
    cobertura.set("version", "1")

    total_lines_valid = 0
    total_lines_covered = 0
    total_branches_valid = 0
    total_branches_covered = 0

    packages_el = ET.SubElement(cobertura, "packages")

    for package in root.findall("package"):
        pkg_name = package.get("name", "").replace("/", ".")
        pkg_el = ET.SubElement(packages_el, "package")
        pkg_el.set("name", pkg_name)

        classes_el = ET.SubElement(pkg_el, "classes")

        pkg_lines_valid = 0
        pkg_lines_covered = 0
        pkg_branches_valid = 0
        pkg_branches_covered = 0

        for sourcefile in package.findall("sourcefile"):
            filename = package.get("name", "") + "/" + sourcefile.get("name", "")

            class_names = [
                cls.get("name", "").replace("/", ".")
                for cls in package.findall("class")
                if cls.get("sourcefilename") == sourcefile.get("name")
            ]

            class_el = ET.SubElement(classes_el, "class")
            class_el.set(
                "name",
                class_names[0]
                if class_names
                else pkg_name + "." + sourcefile.get("name", "").replace(".java", ""),
            )
            class_el.set(
                "filename",
                source_root + "/" + filename if source_root else filename,
            )

            lines_el = ET.SubElement(class_el, "lines")

            cls_lines_valid = 0
            cls_lines_covered = 0
            cls_branches_valid = 0
            cls_branches_covered = 0

            for line in sourcefile.findall("line"):
                line_el = ET.SubElement(lines_el, "line")
                line_el.set("number", line.get("nr", "0"))

                ci = int(line.get("ci", "0"))
                mb = int(line.get("mb", "0"))
                cb = int(line.get("cb", "0"))

                hits = 1 if ci > 0 else 0
                line_el.set("hits", str(hits))

                cls_lines_valid += 1
                if hits:
                    cls_lines_covered += 1

                if mb + cb > 0:
                    total = mb + cb
                    line_el.set("branch", "true")
                    line_el.set(
                        "condition-coverage",
                        f"{int(100 * cb / total)}% ({cb}/{total})",
                    )
                    cls_branches_valid += total
                    cls_branches_covered += cb
                else:
                    line_el.set("branch", "false")

            class_el.set(
                "line-rate",
                str(cls_lines_covered / cls_lines_valid) if cls_lines_valid else "0",
            )
            class_el.set(
                "branch-rate",
                str(cls_branches_covered / cls_branches_valid)
                if cls_branches_valid
                else "0",
            )

            pkg_lines_valid += cls_lines_valid
            pkg_lines_covered += cls_lines_covered
            pkg_branches_valid += cls_branches_valid
            pkg_branches_covered += cls_branches_covered

        pkg_el.set(
            "line-rate",
            str(pkg_lines_covered / pkg_lines_valid) if pkg_lines_valid else "0",
        )
        pkg_el.set(
            "branch-rate",
            str(pkg_branches_covered / pkg_branches_valid)
            if pkg_branches_valid
            else "0",
        )

        total_lines_valid += pkg_lines_valid
        total_lines_covered += pkg_lines_covered
        total_branches_valid += pkg_branches_valid
        total_branches_covered += pkg_branches_covered

    cobertura.set(
        "line-rate",
        str(total_lines_covered / total_lines_valid) if total_lines_valid else "0",
    )
    cobertura.set(
        "branch-rate",
        str(total_branches_covered / total_branches_valid)
        if total_branches_valid
        else "0",
    )
    cobertura.set("lines-valid", str(total_lines_valid))
    cobertura.set("lines-covered", str(total_lines_covered))
    cobertura.set("branches-valid", str(total_branches_valid))
    cobertura.set("branches-covered", str(total_branches_covered))
    cobertura.set("complexity", "0")
    cobertura.set("timestamp", "0")

    tree = ET.ElementTree(cobertura)
    ET.indent(tree, space="  ")
    tree.write(sys.stdout.buffer, xml_declaration=True, encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <jacoco.xml> [source-root]", file=sys.stderr)
        sys.exit(1)
    convert(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "")
