#!/usr/bin/env python3
"""SỬA ĐƯỜNG DẪN VIDEO TRONG splits.json KHI ĐÃ CHUYỂN THƯ MỤC PROJECT / DATASET.

splits.json lưu đường dẫn tuyệt đối lúc chia tập (vd D:\\New folder\\ocsvm_anomaly_fpga\\data\\...).
Chuyển project sang chỗ khác thì bước nào cần đọc lại video gốc sẽ báo "Không mở được video".

    python scripts/fix_split_paths.py                                  # tự đoán: thay phần trước \\data\\ bằng thư mục project hiện tại
    python scripts/fix_split_paths.py --old "D:\\New folder\\ocsvm_anomaly_fpga" --new "D:\\datn\\ocsvm_anomaly_fpga"
    python scripts/fix_split_paths.py --check                          # chỉ kiểm tra, không ghi

Sửa mọi outputs*/**/splits.json (giữ bản cũ thành splits.json.bak). Cách chia tập KHÔNG đổi – chỉ đổi đường dẫn.
"""
import argparse
import json
import os
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def remap(p, old, new):
    if not p:
        return p
    s = str(p)
    if old and s.lower().startswith(old.lower()):
        return new.rstrip("\\/") + s[len(old):].replace("\\", os.sep).replace("/", os.sep)
    return s


def guess_old(paths):
    """Phần đứng trước '\\data\\' (hoặc '/data/') trong đường dẫn cũ."""
    for s in paths:
        s2 = s.replace("/", "\\")
        k = s2.lower().find("\\data\\")
        if k > 0:
            return s[:k]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--old", default=None, help="tiền tố cũ (mặc định tự đoán: phần trước \\data\\)")
    ap.add_argument("--new", default=str(ROOT), help="tiền tố mới (mặc định thư mục project này)")
    ap.add_argument("--check", action="store_true", help="chỉ kiểm tra file nào tồn tại, không ghi")
    a = ap.parse_args()

    files = sorted(p for d in ROOT.glob("outputs*") if d.is_dir() for p in d.rglob("splits.json"))
    if not files:
        print("Không thấy splits.json nào trong outputs*/")
        return
    for f in files:
        raw = json.loads(f.read_text(encoding="utf-8"))
        vids = [d["path"] for v in raw.values() for d in v]
        old = a.old or guess_old(vids)
        new_raw = {k: [{"path": remap(d["path"], old, a.new),
                        "labels": remap(d.get("labels"), old, a.new) if d.get("labels") else None} for d in v]
                   for k, v in raw.items()}
        before = sum(Path(p).exists() for p in vids)
        after = sum(Path(d["path"]).exists() for v in new_raw.values() for d in v)
        print(f"{f.relative_to(ROOT)}: {len(vids)} video · tồn tại trước {before} → sau {after}   ({old} → {a.new})")
        if after < len(vids):
            miss = [d["path"] for v in new_raw.values() for d in v if not Path(d["path"]).exists()][:3]
            print(f"   vẫn thiếu {len(vids) - after}, ví dụ: {miss}")
        if not a.check and old and new_raw != raw and after <= before:
            print("   KHÔNG ghi: sau khi sửa không tìm thấy thêm video nào – kiểm tra lại --new (thư mục chứa data\\)")
        elif not a.check and old and new_raw != raw:
            shutil.copy(f, f.with_suffix(".json.bak"))
            f.write_text(json.dumps(new_raw, ensure_ascii=False, indent=1), encoding="utf-8")
            print("   đã ghi (bản cũ: splits.json.bak)")


if __name__ == "__main__":
    main()
