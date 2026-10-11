"""Generate editable experimental notes and a visually checked PDF. No hardware access."""
import csv, json, math, sys, hashlib
from pathlib import Path
from xml.sax.saxutils import escape
import numpy as np
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
from reportlab.lib.pagesizes import A4

out = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent
data = json.loads((out/'analysis.json').read_text(encoding='utf-8'))
groups = {(s['condition'],s['name']):s for s in data['summaries']}
def ms(s,key,d=3):
    v=s['metrics'][key]
    return f"{v['mean']:.{d}f} ± {v['sd']:.{d}f}"
def mean(c,m,k): return groups[c,m]['metrics'][k]['mean']
def improvement(c,m,k): return 100*(1-mean(c,m,k)/mean(c,'BASE',k))

# Independent implementation: parse selected raw ranges, then use NumPy directly.
verification=[]
for s in data['selected']:
    raw=Path(s['source'])
    if not raw.exists(): raw=Path.cwd()/'01_原始数据'/s['condition']/s['filename']
    b=raw.read_bytes()
    assert hashlib.sha256(b).hexdigest()==s['sourceSHA256']
    rows=list(csv.reader(b.decode('utf-8-sig').splitlines()))
    lo,hi=s['csvLines']; rs=np.array(rows[lo-1:hi],dtype=np.float64)
    mot=rs[(rs[:,2]>=2)&(rs[:,2]<=5)]
    assert len(mot)==400
    errors=(mot[:,5:7]-mot[:,3,None])*1000
    sync=(mot[:,5]-mot[:,6])*1000
    v={'trackingRMSE_mm':float(np.sqrt(np.mean(errors**2))),
       'syncRMS_mm':float(np.sqrt(np.mean(sync**2))),
       'peakTracking_mm':float(np.max(np.abs(errors))),
       'peakSync_mm':float(np.max(np.abs(sync)))}
    for k,val in v.items(): assert abs(val-s['metrics'][k])<1e-12,(s['id'],k,val,s['metrics'][k])
    verification.append({'id':s['id'],'metrics':v,'sourceSHA256':s['sourceSHA256']})
for g in data['summaries']:
    for k in ['trackingRMSE_mm','syncRMS_mm']:
        vs=[v['metrics'][k] for v in verification if v['id'].startswith(g['condition']+'_'+g['name']+'_')]
        assert len(vs)==3
        assert abs(float(np.mean(vs))-g['metrics'][k]['mean'])<1e-12
        assert abs(float(np.std(vs,ddof=1))-g['metrics'][k]['sd'])<1e-12
(out/'independent_verification.json').write_text(json.dumps({'passed':True,'implementation':'Python csv + NumPy, independent of JS metric formulas','trials':verification,'absoluteTolerance':1e-12},ensure_ascii=False,indent=2),encoding='utf-8')
if '--verify-only' in sys.argv:
    print('PASS: independent NumPy metrics, group mean and sample SD, source SHA256')
    sys.exit(0)

intro='本次整理仅包含实验5正式 A、B 两工况：BASE、普通 BLNN、约束感知 CA_BLNN，每条件3次，共18个完整分析样本。保留23份正式原始 CSV（A 14份、B 9份）。不修改前四组封档、仿真快照或原始采集文件。此版本为整理快照，不是整篇论文或全部实验的最终封档。'
protocol='两工况均按操作者所述无附加砝码、2 mm 往返。A 左/右指令界为1200/1200 counts，B 为1200/800 counts。counts 是控制指令计数，不是实测电流安培；I104=0 仅核对固件载荷标签，不是质量测量。两工况同时改变不对称性和总指令预算（2400降至2000），不能将差异仅归因于不对称性。'
validity='18个入选片段均核对107通道、HW6、固件构建/方法、限幅和载荷标签，完整经历 phase 0→6，故障为0，末帧两轴转速和指令为0，phase 6后尾段不少于5秒。阶段2-5均为400帧，名义间隔40 ms，动作时间4.00-19.96 s，无运动/稳定段缺帧；未插值、平滑或按性能挑选片段。通过的是记录完整性及已记录保护状态，不等于所有尝试均成功或所有未采样时刻均已验证。'
definitions=[
 '双轴跟踪 RMSE = sqrt(mean((eL^2+eR^2)/2))，eL=(xL-reference)×1000，eR=(xR-reference)×1000，单位 mm。',
 '同步 RMS = sqrt(mean((xL-xR)^2))×1000，单位 mm。跟踪峰值取两轴绝对误差最大值，同步峰值取绝对同步误差最大值。',
 '各次误差指标在同一阶段2-5的400帧计算。统计量是3次独立记录指标的均值与样本标准差（分母 n-1），不是将1200帧当成1200个独立实验；不作显著性或置信区间声明。',
 '终点误差取所选完整片段的末帧位置相对归零原点的绝对值；不等于统一时刻的动态稳态误差，尾段长度不一。',
 '指令 RMS/峰值和采样触边占比使用阶段2-5；占比阈值为 abs(command) >= limit-1。I61/I62是固件控制活动段累计硬边界次数，不与遥测采样占比混用。',
 '学习系数均值/零学习比例和计算耗时样本均值/P95使用 I106=1 的阶段1-5（20秒）。图4只画阶段2-5（16秒），两者统计窗口不同。',
 'I42与I60为固件累计核心/整周期峰值耗时（us）。表中峰值均值是3次累计峰值的平均，并非完整最坏执行时间保证；40 ms遥测的样本耗时也不是全部1 ms周期分布。',
 '图2展示双轴误差平均值 (eL+eR)/2 与有符号同步误差，不是双轴合并 RMSE。细线为3次原始运行，粗线为相同时间格点均值。'
]
performance=(f"A工况，普通 BLNN 相对 BASE 的平均跟踪 RMSE 降低{improvement('A','BLNN','trackingRMSE_mm'):.1f}%，CA_BLNN降低{improvement('A','CA_BLNN','trackingRMSE_mm'):.1f}%；但 CA_BLNN 平均同步 RMS 为{mean('A','CA_BLNN','syncRMS_mm'):.3f} mm，高于 BASE 的{mean('A','BASE','syncRMS_mm'):.3f} mm。B工况，CA_BLNN 相对 BASE 跟踪 RMSE 降低{improvement('B','CA_BLNN','trackingRMSE_mm'):.1f}%，同步 RMS 降低{improvement('B','CA_BLNN','syncRMS_mm'):.1f}%，普通 BLNN在这两个平均指标上仍低于 CA_BLNN。不能写成 CA_BLNN 全面最优。")
boundary='全部18次的 I61/I62 累计硬边界计数为0，阶段2-5的采样触边占比也均为0。左轴采样指令绝对峰值为615-720 counts，右轴为606-699 counts，B右界800未被触及。因此，本批证明短行程下的实物实现与基础跟踪表现，不能独立证明强执行器饱和条件下的抗饱和优势。图3直接保留配置边界与实际指令之间的距离。'
mechanism=(f"CA_BLNN活动段平均学习系数：A {mean('A','CA_BLNN','sampledLearningMean'):.3f}，B {mean('B','CA_BLNN','sampledLearningMean'):.3f}；采样零学习占比：A {mean('A','CA_BLNN','sampledLearningZero_pct'):.2f}%，B {mean('B','CA_BLNN','sampledLearningZero_pct'):.2f}%。普通 BLNN 活动段学习系数固定为1。右网络末节点数均值：普通 BLNN A {mean('A','BLNN','rightNodes'):.2f}、B {mean('B','BLNN','rightNodes'):.2f}；CA_BLNN A {mean('A','CA_BLNN','rightNodes'):.2f}、B {mean('B','CA_BLNN','rightNodes'):.2f}。这表明门控/增长机制确有活动，不证明其单独导致性能提升。全部入选运行权值投影次数均为0，不能声称普通 BLNN 已发散而 CA 避免了发散。")
provenance='同一 run 编号只占一个分析位置；优先使用满足完整性要求的指定文件，不以最小 RMSE 选样。三对带 _full 的文件在共同序列号上的数值行并不一致，不能仅凭后缀称为“同一次历史补导出”；本整理称为替代完整记录，真实重采/导出来源尚不能由CSV单独确认。全部对应旧文件保留但不重复计入统计，不据此推算独立尝试总数或成功率。'
warning='A_CA_BLNN_2 在 phase 0 的序列间隔缺4帧；A_CA_BLNN_3 在 phase 6 缺1帧，均不在阶段2-5比较窗口内，作为警告保留。A_BLNN_2 的首行是另一构建的非活动末状态，A_BLNN_3和B_BASE_1的首行识别号为0，按精确CSV行范围排除，原始文件未清洗或覆盖。'
implementation='实物固件使用已冻结的 HW6 正式实现。CA 包含整数可行分配、NN增量300 counts界、变化率/扇区/速度保护及学习门控；与冻结 MATLAB v3 仿真及参考英文论文不是逐公式逐参数一对一实现。BASE/BLNN/CA比较应称本项目三个实物控制版本；论文应明确嵌入式适配差异，不把本批描述为原论文数据的完全复现。'
limitations=[boundary,
 '每条件仅3次完整记录；可作描述性对照，证据范围仅为所测无附加砝码、2 mm轨迹及这两组指令界。不能由此推出重载、长行程或所有电池状态下的效果。',
 '电压和温度保留在逐次指标中。文件保存时间不能证明随机化或轮换执行顺序，未将运行顺序、温度、电池状态的影响解释为算法因果效果。',
 '记录合格不等于整个试验历史可靠性为100%。本包没有所有独立尝试的完整现场日志，不能计算全尝试成功率。',implementation,
 'CA_BLNN不是“使用宽度学习网络”本身的原创网络证明。可据本项目方法说明讨论可行域、门控和嵌入式部署贡献，但强饱和性能论据须区分仿真与本批实物证据；不得写成全球首创或所有指标最好。'
]
paper=(f"为验证控制器的实物可运行性，在双驱机构上开展无附加砝码、2 mm往返实验，设置对称指令界 A（1200/1200 counts）和不对称指令界 B（1200/800 counts），分别对 BASE、普通 BLNN 和约束感知 CA_BLNN 采集3次完整记录。以阶段2-5的400帧计算双轴跟踪均方根误差和同步均方根误差，并报告重复实验指标的均值与样本标准差。在 B 工况下，CA_BLNN 相对 BASE 的上述两项平均误差分别降低{improvement('B','CA_BLNN','trackingRMSE_mm'):.1f}%和{improvement('B','CA_BLNN','syncRMS_mm'):.1f}%，但普通 BLNN 在这两项指标上更低；在 A 工况下，CA_BLNN 的同步误差未优于 BASE。两工况下 CA_BLNN 的学习门控和网络增长机制均呈现记录可见的活动。由于本批所有硬边界累计计数均为0，结论限于短行程实物运行与跟踪验证，不据此推断强饱和条件下的优势。")
figs=[('Fig01_individual_metrics','图1 各次误差指标与均值/样本标准差','圆点为3次运行，方块为均值，误差棒为样本标准差；A/B使用可比较坐标尺度，不表示置信区间。'),
      ('Fig02_error_time_series','图2 平均轴跟踪误差与同步误差时序','阶段2-5；细线3次、粗线均值；mean-axis error与合并双轴RMSE不是同一指标，无插值或平滑。'),
      ('Fig03_final_commands','图3 最终指令及配置硬边界','阶段2-5的遥测最终指令，不是电流传感器实测值；灰虚线为配置边界。'),
      ('Fig04_learning_and_nodes','图4 学习系数和右网络节点时序','阶段2-5的门控/增长活动；不将节点减少单独解释为精度提升。')]

md=['# 实验5：A/B正式实物数据整理与论文说明','', '整理日期：2026-10-11；状态：18个完整分析样本整理完成；原始文件保留。','', '## 1. 范围与实验条件','',intro,'',protocol,'',validity,'','## 2. 指标与统计口径','']
md += ['- '+t for t in definitions]
md += ['', '## 3. 结果：均值 ± 样本标准差','', '| 工况 | 方法 | n | 双轴跟踪 RMSE / mm | 同步 RMS / mm |','|---|---|---:|---:|---:|']
for s in data['summaries']: md.append(f"| {s['condition']} | {s['name']} | {s['n']} | {ms(s,'trackingRMSE_mm')} | {ms(s,'syncRMS_mm')} |")
md += ['',performance,'',boundary,'',mechanism,'','## 4. 源文件映射与排除规则','',provenance,'',warning,'', '| 分析位置 | 构建 | 文件（原名不变） | 入选CSV行（含首尾，表头为1） | phase6后尾段/s |','|---|---:|---|---:|---:|']
for s in data['selected']: md.append(f"| {s['id']} | {s['build']} | {s['filename']} | {s['csvLines'][0]}-{s['csvLines'][1]} | {s['metrics']['postStop_s']:.2f} |")
chosen={s['filename'] for s in data['selected']}
md += ['', '### 未纳入统计的5份正式A文件（保留原件）','']
excluded=[]
for e in data['audit']:
    if e['filename'] not in chosen:
        details=[]
        for r in e['runs']:
            for p in r['summary'].get('problems',[]):
                if p['reason']=='post-phase6 shorter than 5 seconds': details.append(f"phase6尾段{p['seconds']:.3f}s，不足5s")
                elif p['reason']=='motion sample gap': details.append(f"phase{p['phase']}缺{p['missingFrames']}帧")
                elif p['reason']=='incomplete motion/settling phase': details.append(f"phase{p['phase']}样本不全")
                else: details.append(p['reason'])
        reason='；'.join(details)
        excluded.append((e['filename'],reason))
        md.append(f"- `{e['filename']}`：{reason}。")
md += ['', '未将这些技术性不完整导出自动判为独立失败试验，也不将替代文件自动判为同一次补导出。', '', '## 5. 论文图说明','']
for stem,title,cap in figs: md += [f'### {title}','',f'![{title}](../04_论文图/{stem}.png)','',cap,'']
md += ['## 6. 机制、限制与论述边界','']+['- '+x for x in limitations]
md += ['', '## 7. 可用于论文结果段的初稿','',paper,'', '## 8. 固件与复现','',
       'A构建：BASE 61010、BLNN 61011、CA_BLNN 61012；B构建：63010、63011、63012。六套Keil工程与对应HEX均按已有发布清单验证源文件与HEX SHA256后复制。原发布清单是准备时快照，其中采集数量字段可能仍为0；本次完成情况以本报告及本次manifest为准，不改历史发布清单。','',
       '原始CSV不改名、不覆盖，文件SHA256和精确入选行范围保存在统计表与analysis.json。02_处理后数据包含完整入选片段（仍为107列）及400帧派生轨迹，均明确标为派生数据。','',
       '06_复现脚本提供analyze.mjs、冻结校验器、表格/图表/PDF构建脚本和reproduce.ps1。分析本身只需Node；表格依赖@oai/artifact-tool，PNG依赖sharp，PDF与独立校验依赖Python NumPy、ReportLab。包装不携带第三方运行库；本机一键脚本使用已安装Codex运行库，可生成新的复算目录，不覆盖本整理结果。异机需安装相应依赖并显式指定运行路径。','',
       'Python NumPy独立复核18次跟踪RMSE、同步RMS、两项峰值和6组均值/样本标准差，与JS计算差值小于1e-12；Excel公式复核通过，无公式错误。PDF逐页与4幅论文PNG均已进行视觉检查。','',
       'README说明整理包结构；manifest.json和SHA256SUMS.txt用于校验。整理仅固定本批数据/代码副本，不自动上传Git、不重烧固件、不删除原件。','']
(out/'实验5_A_B实验结果与论文说明.md').write_text('\n'.join(md),encoding='utf-8')

font_path=Path(sys.argv[2]) if len(sys.argv)>2 else Path('C:/Windows/Fonts/simsun.ttc')
pdfmetrics.registerFont(TTFont('ReportChinese',str(font_path),subfontIndex=0))
body=ParagraphStyle('Body',fontName='ReportChinese',fontSize=10.3,leading=16,spaceAfter=8,wordWrap='CJK',alignment=TA_LEFT)
titleStyle=ParagraphStyle('Title',parent=body,fontSize=19,leading=26,spaceAfter=13)
heading=ParagraphStyle('Heading',parent=body,fontSize=13,leading=19,spaceBefore=5,spaceAfter=9)
small=ParagraphStyle('Small',parent=body,fontSize=8.8,leading=13,spaceAfter=6)
cell=ParagraphStyle('Cell',parent=body,fontSize=9,leading=13,spaceAfter=0)
story=[]
def p(text,style=body): story.append(Paragraph(escape(text).replace('\n','<br/>'),style))
def h(text): p(text,heading)
def table(rows,widths,padding=5):
    t=Table([[Paragraph(escape(str(v)),cell) for v in row] for row in rows],colWidths=widths,repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#E8EEF2')),('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor('#9AA7AF')),('LINEBELOW',(0,1),(-1,-1),.3,colors.HexColor('#DDE2E5')),('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),padding),('BOTTOMPADDING',(0,0),(-1,-1),padding),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6)]))
    story.extend([t,Spacer(1,10)])
def figure(stem): story.append(Image(str(out/'论文图'/f'{stem}.png'),width=499,height=499*760/1040));story.append(Spacer(1,7))
p('实验5正式实物实验整理',titleStyle)
p('A/B工况 · 三方法 · 每条件3次 · 2026-10-11',small)
p(intro);h('实验设置与验收');p(protocol);p(validity)
h('主要结果（均值 ± 样本标准差，n=3）')
rows=[['工况','方法','跟踪RMSE / mm','同步RMS / mm']]
for s in data['summaries']: rows.append([s['condition'],s['name'],ms(s,'trackingRMSE_mm'),ms(s,'syncRMS_mm')])
table(rows,[42,100,178,179]);p(performance);p('重要边界：全部入选运行累计硬边界计数为0，不能将本批单独称为强饱和实物验证。',small)
story.append(PageBreak())
h('源文件处理与统计口径');p(provenance);p(warning)
for t in definitions[:6]: p(t,small)
h('18个完整分析片段')
rows=[['分析位置','构建','CSV行','停机尾段 / s']]
for s in data['selected']: rows.append([s['id'],s['build'],f"{s['csvLines'][0]}-{s['csvLines'][1]}",f"{s['metrics']['postStop_s']:.2f}"])
table(rows,[175,80,126,118],padding=2.5);p('完整文件名、源SHA256、5份排除文件的逐条理由见可编辑说明和Excel文件核验页。原始CSV保留不变。',small)
story.append(PageBreak())
h('论文图1：误差指标对照');figure('Fig01_individual_metrics');p(figs[0][2],small)
p(performance);h('建议用于论文的结果段');p(paper)
story.append(PageBreak())
h('论文图3：实际指令与配置边界');figure('Fig03_final_commands');p(figs[2][2],small)
p(boundary);h('学习与节点活动');p(mechanism);p('另附图2误差时序与图4学习/节点时序（SVG和高分辨率PNG）。图4使用阶段2-5，活动段统计均值使用阶段1-5，窗口不能混写。',small)
story.append(PageBreak())
h('论文结论的使用范围')
for t in limitations[1:]: p(t)
p(definitions[6],small)
h('代码冻结副本与复现');p('六套固件标识：A BASE/BLNN/CA为61010/61011/61012；B为63010/63011/63012。复制前逐一核对发布清单源文件和HEX SHA256。历史准备清单中的采集数量字段不作为当前进度。')
p('原始数据、完整入选片段、400帧派生轨迹、逐次指标、Excel、4幅论文图和六套代码均分目录保存。各原文件可通过SHA256追溯；入选行和替代记录关系均有明确说明。')
p('独立Python NumPy复核18次主要误差指标及6组均值/样本标准差，误差小于1e-12；Excel公式复算无错误。复现脚本可新建复算目录，不覆盖原始数据或本次结果。第三方库不打包，本机运行库路径写在脚本中，异机需另行配置。')
p('本批完成的是“数据整理与可追溯快照”。不自动上传Git、不操作硬件、不删除原文件，也不等于所有论文创新主张已被证明。',small)
def footer(c,doc):
    c.saveState();c.setStrokeColor(colors.HexColor('#CBD1D5'));c.line(48,39,A4[0]-48,39)
    c.setFont('ReportChinese',8);c.drawString(48,25,'实验5 A/B正式数据整理 | 2026-10-11');c.drawRightString(A4[0]-48,25,str(doc.page));c.restoreState()
dest=out/'实验5_A_B实验结果与论文说明.pdf'
doc=SimpleDocTemplate(str(dest),pagesize=A4,rightMargin=48,leftMargin=48,topMargin=42,bottomMargin=52,title='实验5 A/B正式实物实验整理',author='实验数据整理')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
print(json.dumps({'pdf':str(dest),'md':str(out/'实验5_A_B实验结果与论文说明.md'),'independentVerification':'PASS'},ensure_ascii=False))
