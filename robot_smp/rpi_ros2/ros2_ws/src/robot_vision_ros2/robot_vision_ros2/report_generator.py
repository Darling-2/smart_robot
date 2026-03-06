#!/usr/bin/env python3
# coding=utf-8
"""
巡检报告生成器
功能：从 inspection_log.csv + images/ 生成 HTML 巡检报告
用途：毕设答辩展示
"""

import os
import csv
import base64
import argparse
from datetime import datetime
from collections import Counter


def load_csv(csv_path: str):
    rows = []
    try:
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
    except Exception as e:
        print(f'[WARN] Failed to parse CSV: {e}')
    return rows


def img_to_base64(img_path: str) -> str:
    try:
        with open(img_path, 'rb') as f:
            return base64.b64encode(f.read()).decode('utf-8')
    except Exception as e:
        print(f'[WARN] Failed to read image {img_path}: {e}')
        return ''


def generate_report(save_dir: str, output_path: str):
    csv_path = os.path.join(save_dir, 'inspection_log.csv')
    img_dir = os.path.join(save_dir, 'images')

    if not os.path.exists(csv_path):
        print(f'[ERROR] CSV not found: {csv_path}')
        return

    rows = load_csv(csv_path)
    total = len(rows)

    if total == 0:
        print('[WARN] No detection records found, generating empty report.')

    # 统计
    class_counter = Counter()
    waypoint_counter = Counter()
    conf_sum = 0.0
    for r in rows:
        class_counter[r.get('class_name', 'unknown')] += 1
        wp = r.get('waypoint', '')
        if wp:
            waypoint_counter[wp] += 1
        try:
            conf_sum += float(r.get('conf', 0))
        except ValueError:
            pass
    avg_conf = conf_sum / total if total > 0 else 0

    # 巡检时长
    duration_str = '-'
    if total >= 2:
        try:
            t0 = datetime.strptime(rows[0].get('timestamp', ''), '%Y-%m-%d %H:%M:%S')
            t1 = datetime.strptime(rows[-1].get('timestamp', ''), '%Y-%m-%d %H:%M:%S')
            delta = t1 - t0
            mins = int(delta.total_seconds() // 60)
            secs = int(delta.total_seconds() % 60)
            duration_str = f'{mins}分{secs}秒'
        except Exception:
            pass

    # 图片列表
    images = []
    if os.path.isdir(img_dir):
        images = sorted([f for f in os.listdir(img_dir) if f.endswith(('.jpg', '.png'))])

    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # 生成 HTML
    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>巡检报告</title>
<style>
  body {{ font-family: "Microsoft YaHei", sans-serif; max-width: 900px; margin: 40px auto; padding: 0 20px; }}
  h1 {{ text-align: center; color: #2c3e50; }}
  .summary {{ background: #ecf0f1; padding: 20px; border-radius: 8px; margin: 20px 0; }}
  .summary td {{ padding: 6px 16px; }}
  table.log {{ width: 100%; border-collapse: collapse; margin: 20px 0; }}
  table.log th, table.log td {{ border: 1px solid #bdc3c7; padding: 8px; text-align: center; }}
  table.log th {{ background: #3498db; color: #fff; }}
  table.log tr:nth-child(even) {{ background: #f9f9f9; }}
  .gallery {{ display: flex; flex-wrap: wrap; gap: 10px; }}
  .gallery img {{ max-width: 280px; border: 2px solid #3498db; border-radius: 4px; }}
  .footer {{ text-align: center; color: #999; margin-top: 40px; font-size: 12px; }}
</style>
</head>
<body>
<h1>🔍 巡检报告</h1>
<div class="summary">
<table>
  <tr><td><b>报告生成时间</b></td><td>{now}</td></tr>
  <tr><td><b>巡检时长</b></td><td>{duration_str}</td></tr>
  <tr><td><b>检测总数</b></td><td>{total}</td></tr>
  <tr><td><b>平均置信度</b></td><td>{avg_conf:.4f}</td></tr>
  <tr><td><b>类别分布</b></td><td>{dict(class_counter)}</td></tr>
  <tr><td><b>巡检点覆盖</b></td><td>{dict(waypoint_counter) if waypoint_counter else '无巡检点数据'}</td></tr>
  <tr><td><b>保存图片数</b></td><td>{len(images)}</td></tr>
</table>
</div>

<h2>📋 检测记录</h2>
<table class="log">
<tr><th>时间</th><th>巡检点</th><th>类别</th><th>置信度</th><th>边界框</th></tr>
"""

    for r in rows[-100:]:  # 最多展示最近100条
        cls = r.get('class_name', '')
        # 异常/缺陷类别高亮显示
        td_style = ' style="color:#e74c3c;font-weight:bold"' if cls in ('defect', 'abnormal') else ''
        html += f"<tr><td>{r.get('timestamp','')}</td>"
        html += f"<td>{r.get('waypoint','')}</td>"
        html += f"<td{td_style}>{cls}</td><td>{r.get('conf','')}</td>"
        html += f"<td>{r.get('bbox_xyxy','')}</td></tr>\n"

    html += "</table>\n"

    if images:
        html += "<h2>📷 检测截图（最近20张）</h2>\n<div class='gallery'>\n"
        for img_name in images[-20:]:
            img_path = os.path.join(img_dir, img_name)
            b64 = img_to_base64(img_path)
            html += f'<img src="data:image/jpeg;base64,{b64}" title="{img_name}"/>\n'
        html += "</div>\n"

    html += f"""
<div class="footer">本报告由巡检系统自动生成 | {now}</div>
</body></html>
"""

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f'[INFO] Report saved: {output_path}')


def main():
    parser = argparse.ArgumentParser(description='Generate inspection report')
    parser.add_argument('--save-dir', type=str, default='/tmp/inspection', help='Inspection data directory')
    parser.add_argument('--output', type=str, default='', help='Output HTML path')
    args = parser.parse_args()

    if not args.output:
        args.output = os.path.join(args.save_dir, 'inspection_report.html')

    generate_report(args.save_dir, args.output)


if __name__ == '__main__':
    main()
