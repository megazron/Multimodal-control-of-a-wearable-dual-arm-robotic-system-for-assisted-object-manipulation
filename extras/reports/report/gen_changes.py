import subprocess,re,collections,os
def esc(s):
    s=s.replace('\\','\\textbackslash ')
    for a,b in [('_','\\_'),('%','\\%'),('&','\\&'),('#','\\#'),('$','\\$'),
                ('{','\\{'),('}','\\}'),('~','\\textasciitilde '),('^','\\textasciicircum ')]:
        s=s.replace(a,b)
    s=s.replace('—','---').replace('–','--').replace('“','``').replace('”',"''")
    s=s.replace('‘','`').replace('’',"'").replace('…','\\ldots ').replace('°','$^\\circ$')
    s=s.replace('±','$\\pm$').replace('×','$\\times$').replace('≥','$\\ge$').replace('≤','$\\le$')
    s=s.replace('→','$\\rightarrow$').replace('·','$\\cdot$').replace('≈','$\\approx$')
    s=s.replace('π','$\\pi$').replace('µ','\\textmu ').replace('−','$-$').replace('²','$^2$')
    s=s.replace(' ',' ').replace('‑','-').replace('✓','yes').replace('✗','no')
    s=re.sub(r'`([^`]+)`', r'\\code{\1}', s)
    return s

REC='\x02'; FLD='\x01'
raw=subprocess.check_output(
    ['git','log','--reverse','--date=short',
     '--format=' + REC + '%h' + FLD + '%ad' + FLD + '%s' + FLD + '%b'],text=True)
commits=[]
for block in raw.split(REC):
    if not block.strip(): continue
    parts=block.split(FLD)
    if len(parts)<4: continue
    h,d,subj,body=parts[0].strip(),parts[1],parts[2],FLD.join(parts[3:])
    body=re.sub(r'Co-Authored-By:.*','',body,flags=re.S).strip()
    commits.append((h,d,subj,body))
print('commits',len(commits))

def condense(body, maxchars=520):
    if not body: return ''
    paras=[p for p in re.split(r'\n\s*\n', body) if p.strip()]
    out=[]
    n=0
    for p in paras:
        t=' '.join(p.split())
        if t.startswith(('*','-','|')) and out: break
        out.append(t); n+=len(t)
        if n>maxchars: break
    s=' '.join(out)
    if len(s)>maxchars:
        cut=s[:maxchars]
        k=cut.rfind('. ')
        s=cut[:k+1] if k>200 else cut.rstrip()+' [...]'
    return s

byday=collections.OrderedDict()
for c in commits: byday.setdefault(c[1],[]).append(c)

o=[]
w=o.append
w(r'\chapter{Every change}\label{ch:changes}')
w(r'All %d commits in order, oldest first, each with its own account of what it '
  r'changed and why. Messages in this repository are written as findings rather '
  r'than as labels, so this chapter doubles as the project diary. Bodies are '
  r'condensed to their opening argument; the full text is in \code{git log}.' % len(commits))
w('')
for day,cs in byday.items():
    w(r'\section*{%s \textnormal{\small\textcolor{srlgrey}{--- %d commit%s}}}'
      % (day, len(cs), '' if len(cs)==1 else 's'))
    w(r'\addcontentsline{toc}{section}{%s}' % day)
    w(r'\begin{longtable}{@{}p{0.055\textwidth}p{0.885\textwidth}@{}}')
    w(r'\endfirsthead\endhead')
    for h,d,subj,body in cs:
        c=condense(body)
        w(r'{\scriptsize\ttfamily %s} & \textbf{%s}%s \\' %
          (esc(h), esc(subj), ('\\newline{\\small ' + esc(c) + '}') if c else ''))
        w(r'\addlinespace[3pt]')
    w(r'\end{longtable}')
    w('')
open('docs/report/gen_changes.tex','w').write('\n'.join(o)+'\n')
print('wrote',len(o),'lines')
