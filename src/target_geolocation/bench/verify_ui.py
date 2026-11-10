#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证「仿真视窗」下拉里是否出现「像素流帧率」这一行（在 Jetson 上运行）。"""
import sys

sys.path.insert(0, '/tmp')
from probe_page import CDP, get_pages     # noqa: E402

JS = r"""
(function(){
  var out = [];
  var rows = [].slice.call(document.querySelectorAll('.dd-row'));
  for (var i = 0; i < rows.length; i++) {
    var t = (rows[i].textContent || '').replace(/\s+/g, ' ').trim();
    if (t.indexOf('像素流') >= 0) {
      var btns = [].slice.call(rows[i].querySelectorAll('button'))
                   .map(function(b){ return b.textContent.trim(); });
      out.push({text: t.slice(0, 40), buttons: btns});
    }
  }
  return {rows: out, stored: localStorage.getItem('ue-addr')};
})()
"""


def main():
    page = next((p for p in get_pages(9334) if p.get('type') == 'page'), None)
    if not page:
        sys.exit('没找到 page')
    cdp = CDP(page['webSocketDebuggerUrl'])
    try:
        r = cdp.js(JS)
        print('页面: %s' % page.get('title'))
        print('\n=== 含「像素流」的配置行 ===')
        if not r['rows']:
            print('  （未找到。下拉可能是展开时才渲染，请手动点开「仿真视窗」菜单确认）')
        for row in r['rows']:
            print('  %-22s 选项: %s' % (row['text'], ' / '.join(row['buttons'])))
        print('\nlocalStorage(ue-addr) = %s' % (r.get('stored') or '(空)'))
    finally:
        cdp.close()


if __name__ == '__main__':
    main()
