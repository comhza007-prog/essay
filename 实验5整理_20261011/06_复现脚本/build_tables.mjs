import fs from 'node:fs/promises';import path from 'node:path';import assert from 'node:assert/strict';
import {Workbook,SpreadsheetFile} from '@oai/artifact-tool';
process.on('uncaughtException',e=>{console.error(e.message);process.exit(1);});
const dir=path.dirname(new URL(import.meta.url).pathname.replace(/^\/(\w:)/,'$1'));
// Decode Chinese Windows paths supplied by the file URL.
const out=process.argv[2]?path.resolve(process.argv[2]):decodeURIComponent(dir),data=JSON.parse(await fs.readFile(path.join(out,'analysis.json'),'utf8'));
const wb=Workbook.create(),sum=wb.worksheets.add('汇总'),trials=wb.worksheets.add('逐次指标'),audit=wb.worksheets.add('文件核验');
const keys=['trackingRMSE_mm','syncRMS_mm','peakTracking_mm','peakSync_mm','terminalMaxAbs_mm','leftCmdRMS_counts','rightCmdRMS_counts','sampledLeftBoundary_pct','sampledRightBoundary_pct','corePeak_us','wholePeak_us','batteryMin_V','leftNodes','rightNodes','sampledLearningMean','sampledLearningZero_pct','postStop_s'];
const headers=['工况','方法','重复','构建','双轴跟踪RMSE mm','同步RMS mm','跟踪峰值 mm','同步峰值 mm','终点最大绝对位置 mm','左指令RMS counts','右指令RMS counts','采样左触边 %','采样右触边 %','累计核心峰值 us','累计整周期峰值 us','最低电压 V','左节点','右节点','采样学习系数均值','采样零学习 %','停机尾段 s','文件','CSV行范围','源文件SHA256'];
const vals=data.selected.map(s=>[s.condition,s.name,s.run,s.build,...keys.map(k=>s.metrics[k]),s.filename,s.csvLines.join('-'),s.sourceSHA256]);
trials.getRange('A2').values=[['正式实物逐次指标']];trials.getRange('A3').values=[['阶段2-5共400帧；双轴RMSE为两轴平方误差的合并均方根；未插值']];
trials.getRange('A5:X23').values=[headers,...vals];trials.getRange('Z5').values=[['原始来源']];trials.getRange('Z6:Z23').values=data.selected.map(s=>[[s.source,'CSV行'+s.csvLines.join('-')].join('；')]);
sum.getRange('A2').values=[['BLNN与约束感知BLNN正式实物对照']];sum.getRange('A3').values=[['无附加砝码 2 mm；A为1200/1200，B为1200/800 counts；每条件3次，描述性均值和样本标准差']];
sum.getRange('A5:K5').values=[['工况','方法','n','跟踪均值 mm','跟踪SD mm','同步均值 mm','同步SD mm','跟踪峰值均值 mm','同步峰值均值 mm','终点绝对误差均值 mm','整周期峰值均值 us']];
for(let i=0;i<6;i++){
 const s=data.summaries[i],r=6+i,a=6+3*i,b=a+2;
 sum.getRange(`A${r}:C${r}`).values=[[s.condition,s.name,s.n]];
 const formulas=[`=AVERAGE('逐次指标'!E${a}:E${b})`,`=STDEV('逐次指标'!E${a}:E${b})`,`=AVERAGE('逐次指标'!F${a}:F${b})`,`=STDEV('逐次指标'!F${a}:F${b})`,`=AVERAGE('逐次指标'!G${a}:G${b})`,`=AVERAGE('逐次指标'!H${a}:H${b})`,`=AVERAGE('逐次指标'!I${a}:I${b})`,`=AVERAGE('逐次指标'!O${a}:O${b})`];
 sum.getRange(`D${r}:K${r}`).formulas=[formulas];
}
sum.getRange('A14').values=[['解释：不做显著性检验；不宣称CA全面优于普通BLNN。']];
sum.getRange('A15').values=[['A/B同时改变不对称性和总指令预算；不能把差异只归因于不对称性。']];
sum.getRange('A16').values=[['40ms采样触边比例与1ms累计触边计数分开；指令counts不是安培。']];
const checks=data.audit.map(e=>[e.condition,e.filename,e.rows,e.wholeFile.eligibleForNumericalComparison?'全文件通过':'见片段或技术问题',e.runs.map(r=>`${r.name}:${r.csvLines.join('-')}:${r.summary.eligibleForNumericalComparison?'通过':r.summary.error??r.summary.problems.map(p=>p.reason).join('/')}`).join('；'),e.unidentifiedRows.length,data.selected.filter(s=>s.filename===e.filename).map(s=>s.id).join('；'),e.sha256]);
audit.getRange('A2').values=[['原始文件及分析范围核验']];audit.getRange('A3').values=[['23个原始文件对应18个分析位置；扩展文件和重采文件不重复计数；原始观测未改动']];
audit.getRange('A5:H28').values=[['工况','文件','行数','全文件状态','片段及技术问题','未知行数','纳入位置','SHA256'],...checks];
audit.getRange('J5').values=[['原始来源']];audit.getRange('J6:J28').values=data.audit.map(e=>[e.file]);
for(const sheet of [sum,trials,audit]){
 sheet.showGridLines=false;const range=sheet===sum?'A2:K16':sheet===trials?'A2:Z23':'A2:J28';
 sheet.getRange(range).format.font={name:'Arial',size:10,color:'#202020'};
 sheet.getRange(range).format.verticalAlignment='center';sheet.getRange(range).format.rowHeight=23;
 sheet.getRange('A2').format.font={name:'Arial',size:14,bold:true,color:'#000000'};
 const header=sheet===sum?'A5:K5':sheet===trials?'A5:X5':'A5:H5';
 sheet.getRange(header).format={fill:'#24415F',font:{name:'Arial',size:10,bold:true,color:'#FFFFFF'},wrapText:true,horizontalAlignment:'center',verticalAlignment:'center',rowHeight:42};
 const body=sheet===sum?'A6:K11':sheet===trials?'A6:X23':'A6:H28';sheet.getRange(body).format.borders={insideHorizontal:{style:'thin',color:'#E4E7EB'}};
 sheet.getRange('A:A').format.columnWidth=9;sheet.getRange('B:B').format.columnWidth=15;
}
sum.getRange('C:C').format.columnWidth=6;sum.getRange('D:K').format.columnWidth=16;sum.getRange('D6:K11').setNumberFormat('0.000');sum.getRange('K6:K11').setNumberFormat('0.0');
trials.getRange('C:D').format.columnWidth=10;trials.getRange('E:U').format.columnWidth=18;trials.getRange('E6:U23').setNumberFormat('0.000');trials.getRange('Q6:R23').setNumberFormat('0');trials.getRange('V:V').format.columnWidth=67;trials.getRange('W:W').format.columnWidth=15;trials.getRange('X:X').format.columnWidth=70;trials.getRange('Z:Z').format.columnWidth=100;trials.freezePanes.freezeRows(5);
audit.getRange('B:B').format.columnWidth=67;audit.getRange('C:C').format.columnWidth=9;audit.getRange('D:D').format.columnWidth=20;audit.getRange('E:E').format.columnWidth=90;audit.getRange('F:G').format.columnWidth=18;audit.getRange('H:H').format.columnWidth=70;audit.getRange('J:J').format.columnWidth=100;audit.freezePanes.freezeRows(5);
wb.recalculate();
const computed=sum.getRange('D6:G11').values;
for(let i=0;i<6;i++)for(const [j,key,stat] of [[0,'trackingRMSE_mm','mean'],[1,'trackingRMSE_mm','sd'],[2,'syncRMS_mm','mean'],[3,'syncRMS_mm','sd']])assert.ok(Math.abs(computed[i][j]-data.summaries[i].metrics[key][stat])<1e-10,'Excel and independent JS summary disagree');
const check=await wb.inspect({kind:'table',range:'汇总!A5:K11',include:'values,formulas',tableMaxRows:7,tableMaxCols:11,maxChars:1600});console.log(check.ndjson);
const errors=await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:50},maxChars:2000});console.log(errors.ndjson);
for(const [sheet,range] of [['汇总','A1:K17'],['逐次指标','A4:I23'],['文件核验','A4:D28']]){
 const preview=await wb.render({sheetName:sheet,range,scale:1.5,format:'png'});await fs.writeFile(path.join(out,'QA_'+sheet+'.png'),new Uint8Array(await preview.arrayBuffer()));
}
await (await SpreadsheetFile.exportXlsx(wb)).save(path.join(out,'实验5_A_B逐次指标与统计.xlsx'));
await fs.writeFile(path.join(out,'table_verification.json'),JSON.stringify({recalculated:true,summaryAgreesWithIndependentJS:true,rows:18,sourceFiles:23},null,2));
const csv=(head,rows)=>'\uFEFF'+[head,...rows].map(row=>row.map(v=>{v=String(v??'');return /[",\r\n]/.test(v)?'"'+v.replaceAll('"','""')+'"':v;}).join(',')).join('\r\n')+'\r\n';
await fs.writeFile(path.join(out,'逐次指标.csv'),csv(['id','condition','method','run','build',...Object.keys(data.selected[0].metrics),'source_csv','source_csv_lines','source_sha256'],data.selected.map(s=>[s.id,s.condition,s.name,s.run,s.build,...Object.values(s.metrics),s.filename,s.csvLines.join('-'),s.sourceSHA256])));
await fs.writeFile(path.join(out,'统计汇总.csv'),csv(['condition','method','n','metric','mean','sample_sd','min','max'],data.summaries.flatMap(s=>Object.entries(s.metrics).map(([k,v])=>[s.condition,s.name,s.n,k,v.mean,v.sd,v.min,v.max]))));
console.log('Created one formula-backed workbook and CSV analysis tables.');
