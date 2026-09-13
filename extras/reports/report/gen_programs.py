import ast,os,glob,re,sys

def esc(s):
    s=s.replace('\\','\\textbackslash ')
    for a,b in [('_','\\_'),('%','\\%'),('&','\\&'),('#','\\#'),('$','\\$'),
                ('{','\\{'),('}','\\}'),('~','\\textasciitilde '),('^','\\textasciicircum ')]:
        s=s.replace(a,b)
    s=s.replace('—','---').replace('–','--').replace('“','``').replace('”',"''")
    s=s.replace('‘','`').replace('’',"'").replace('…','\\ldots ').replace('°','$^\\circ$')
    s=s.replace('±','$\\pm$').replace('×','$\\times$').replace('≥','$\\ge$').replace('≤','$\\le$')
    s=s.replace('→','$\\rightarrow$').replace('·','$\\cdot$').replace('≈','$\\approx$')
    s=s.replace('π','$\\pi$').replace('µ','\\textmu ').replace('−','$-$')
    s=s.replace(' ',' ').replace('‑','-').replace('✓','yes').replace('✗','no')
    return s

def summary(path, maxlen=300):
    src=open(path,encoding='utf-8',errors='ignore').read()
    d=None
    if path.endswith('.py'):
        try: d=ast.get_docstring(ast.parse(src))
        except Exception: d=None
    if d:
        para=d.strip().split('\n\n')[0]
        out=[]
        for ln in para.split('\n'):
            t=ln.strip()
            if not t: continue
            if out and out[-1][-1:] not in '.!?:;,-' and t[:1].isupper():
                out[-1]=out[-1]+'.'
            out.append(t)
        s=' '.join(' '.join(out).split())
    else:
        lines=[]
        for l in src.split('\n')[:30]:
            t=l.strip()
            if t.startswith('#!') or t.startswith('# -*-'): continue
            if t.startswith('#'):
                lines.append(t.lstrip('#').strip())
            elif lines:
                break
        s=' '.join(lines[:5])
    s=s.strip()
    # strip leading "name.py -- " style prefix
    base=os.path.basename(path)
    for pre in (base+' -- ', base+' --- ', base+' - ', base+': ', base+' '):
        if s.startswith(pre): s=s[len(pre):]; break
    if s.lower().startswith(base.lower()):
        s=s[len(base):]
    s=re.sub(r'^[\s\-—:=.]+','',s)
    s=re.sub(r'={4,}',' ',s)
    s=re.sub(r'-{4,}',' ',s)
    s=' '.join(s.split())
    if s[:1].islower() and not s[:1].isdigit():
        s=s[0].upper()+s[1:]
    if len(s)>maxlen:
        cut=s[:maxlen]
        if '. ' in cut: cut=cut[:cut.rfind('. ')+1]
        else: cut=cut[:cut.rfind(' ')]+' ...'
        s=cut
    return s or '(no description in source)'

def nlines(p): return sum(1 for _ in open(p,encoding='utf-8',errors='ignore'))

def brk(name):
    e=esc(name)
    e=e.replace('\\_','\\_\\allowbreak{}').replace('/','/\\allowbreak{}')
    return e

def table(rows, w1=0.30):
    o=[]
    o.append(r'\begin{longtable}{@{}>{\raggedright\arraybackslash}p{%.2f\textwidth}p{%.2f\textwidth}@{}}' % (w1, 0.94-w1))
    o.append(r'\toprule \textbf{Program} & \textbf{What it is} \\ \midrule \endfirsthead')
    o.append(r'\toprule \textbf{Program} & \textbf{What it is} \\ \midrule \endhead')
    for name,ln,desc in rows:
        o.append(r'\file{%s}\newline{\scriptsize\textcolor{srlgrey}{%d lines}} & \small %s \\' %
                 (brk(name), ln, esc(desc)))
        o.append(r'\addlinespace[2pt]')
    o.append(r'\bottomrule')
    o.append(r'\end{longtable}')
    return o

out=[]
w=out.append

PKGS=[('srl_teleop','Teleoperation: master sensing, following, safety, the bridge, the window'),
      ('srl_perception','Cameras, detection, scene understanding, the world model, wearer tracking, the planner'),
      ('srl_autonomy','Grasp generation, intent, arbitration, language and voice'),
      ('srl_experiments','Tasks, conditions, logging, sessions, analysis, the work surface'),
      ('srl_vr_teleop','The Quest bridge, the pose mapper, VR safety'),
      ('srl_vr_autonomy','VR-side intent and arbitration'),
      ('srl_description','The robot description'),
      ('srl_moveit_config','IK, controllers, SRDF')]

w(r'\chapter{Every program: the packages}')
w(r'Every first-party module in \file{src/}, with the description taken from '
  r'the program\textquotesingle s own docstring. Test modules are in '
  r'Chapter~\ref{ch:tests}; launch files are listed at the end of this chapter.')
w('')
tot_files=tot_lines=0
for pkg,blurb in PKGS:
    mods=[]
    for root,dirs,fs in os.walk('src/'+pkg):
        if '/test' in root or '_archive' in root: continue
        for f in sorted(fs):
            if not f.endswith('.py'): continue
            if f in ('setup.py','__init__.py'): continue
            if '/launch' in root: continue
            p=os.path.join(root,f)
            mods.append((os.path.relpath(p,'src/'+pkg), nlines(p), summary(p)))
    if not mods: continue
    mods.sort()
    tl=sum(m[1] for m in mods); tot_files+=len(mods); tot_lines+=tl
    w(r'\section{\texttt{%s}}' % esc(pkg))
    w(r'\textit{%s.} %d modules, %s lines.' % (esc(blurb), len(mods), format(tl,',')))
    w('')
    out.extend(table(mods))
    w('')

# archived experiments
arch=[]
for root,dirs,fs in os.walk('src/srl_experiments/experiments'):
    for f in sorted(fs):
        if f.endswith('.py'):
            p=os.path.join(root,f)
            arch.append((os.path.relpath(p,'src/srl_experiments'), nlines(p), summary(p)))
arch.sort()
w(r'\section{\texttt{srl\_experiments/experiments} --- the task layer and the archive}')
w(r'\textit{The live task set is \file{experiments/abc/}; \file{experiments/\_archive/} '
  r'holds the superseded bimanual and E1--E6 sets, kept because recorded data still '
  r'refers to them.} %d modules.' % len(arch))
w('')
out.extend(table(arch,0.34))
w('')

# launch files
lf=[]
for p in sorted(glob.glob('src/srl_*/launch/*.py')):
    lf.append((p.replace('src/',''), nlines(p), summary(p)))
w(r'\section{Launch files}')
out.extend(table(lf,0.34))
w('')
w(r'\textbf{Total: %d first-party modules, %s lines, excluding tests and launch files.}'
  % (tot_files, format(tot_lines,',')))
w('')

# ---------- scripts ----------
GROUPS=[('measure_','Measurement --- each produces a number and writes a baseline'),
        ('verify_','Verification --- each produces a pass/fail verdict'),
        ('audit_','Audit --- checks the repository against its own declarations'),
        ('sweep_','Sweeps --- explore a parameter space and record the whole surface'),
        ('search_','Searches over configuration space'),
        ('solve_','Solvers --- poses, layouts, transits'),
        ('record_','Recording and capture'),
        ('render_','Rendering'),
        ('probe_','Probes --- small targeted experiments'),
        ('check_','Session checks'),
        ('calibrate_','Calibration'),
        ('diagnose_','Diagnosis'),
        ('inject_','Fault injection'),
        ('pick_','Pick execution'),
        ('srl_','Operator-facing tools and windows'),
        ('scan_','Scanning'),
        ('stage_','Staging'),
        ('analyse_','Analysis'),
        ('cad_','CAD'),
        ('find_','Finders'),
        ('make_','Generators'),
        ('run_','Runners'),
        ('vr_','VR tooling'),
        ('goto_','Motion helpers'),
        ('locate_','Location'),
        ('build_','Builders'),
        ('gate','Gates'),
        ('apply_','Appliers'),
        ('choose_','Choosers'),
        ('drive_','Drivers'),
        ('execute_','Execution'),
        ('read_','Readers'),
        ('score_','Scoring'),
        ('map_','Mapping'),
        ('merge_','Merging'),
        ('mode_','Modes'),
        ('auto_','Automation'),
        ('observer_','Observer'),
        ('handeye_','Hand-eye'),
        ('guarded_','Guarded motion'),
        ('finish_','Completion'),
        ('follower_','Follower control'),
        ('describe_','Description'),
        ('shoot_','Capture'),
        ('sample_','Sampling'),
        ('settle_','Settling'),
        ('survey_','Surveys'),
        ('relook','Re-look'),
        ('retract_','Retraction'),
        ('revalidate_','Revalidation'),
        ('recording_','Recording layout'),
        ('safe_','Safe motion'),
        ('servo_','Servoing'),
        ('sim_','Simulated sessions'),
        ('status_','Status'),
        ('accuracy_','Accuracy'),
        ('break_','Session faults'),
        ('clip_','Clips'),
        ('env_','Environment'),
        ('final5','Archived runs'),
        ('gen_','Generators'),
        ('gripper','Gripper'),
        ('instruct_','Instruction'),
        ('lerobot_','LeRobot'),
        ('pilot_','Pilot runs'),
        ('plan_','Planning'),
        ('pose_','Posing'),
        ('prove_','Proofs'),
        ('virtual_','Virtual hardware'),
        ]
w(r'\chapter{Every program: the scripts}')
w(r'\file{scripts/} holds %d Python programs and %d shell entry points. Every one '
  r'is listed below with its own description. The verb in the name is the '
  r'project\textquotesingle s taxonomy and it is used consistently.'
  % (len(glob.glob('scripts/*.py')), len(glob.glob('scripts/*.sh'))))
w('')
names=sorted(os.path.basename(p) for p in glob.glob('scripts/*.py'))
used=set()
for pre,desc in GROUPS:
    sel=[n for n in names if n.startswith(pre) and n not in used]
    if not sel: continue
    used.update(sel)
    rows=[(n, nlines('scripts/'+n), summary('scripts/'+n)) for n in sel]
    w(r'\section{%s}' % esc(desc))
    out.extend(table(rows,0.28))
    w('')
rest=[n for n in names if n not in used]
if rest:
    rows=[(n, nlines('scripts/'+n), summary('scripts/'+n)) for n in rest]
    w(r'\section{Everything else}')
    out.extend(table(rows,0.28))
    w('')
shs=sorted(os.path.basename(p) for p in glob.glob('scripts/*.sh'))
rows=[(n, nlines('scripts/'+n), summary('scripts/'+n)) for n in shs]
w(r'\section{Shell entry points}')
out.extend(table(rows,0.28))
w('')

# ---------- tests ----------
w(r'\chapter{Every program: the tests}\label{ch:tests}')
w(r'114 test modules, 1\,322 passing tests. Each is listed with what it pins. '
  r'A test name in this project is a sentence, and a great many of them name a '
  r'defect that was once real.')
w('')
bypkg={}
for p in sorted(glob.glob('src/*/test/test_*.py')):
    bypkg.setdefault(p.split('/')[1],[]).append(p)
for pkg in sorted(bypkg):
    rows=[(os.path.basename(p), nlines(p), summary(p)) for p in bypkg[pkg]]
    w(r'\section{\texttt{%s} --- %d modules}' % (esc(pkg), len(rows)))
    out.extend(table(rows,0.32))
    w('')

open('docs/report/gen_programs.tex','w').write('\n'.join(out)+'\n')
print('wrote', len(out), 'lines')
