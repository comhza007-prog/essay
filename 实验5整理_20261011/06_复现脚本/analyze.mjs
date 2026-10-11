import fs from 'node:fs';import path from 'node:path';import crypto from 'node:crypto';import vm from 'node:vm';import assert from 'node:assert/strict';
const root=process.cwd(),out=process.argv[2]?path.resolve(process.argv[2]):path.join(root,'experiment5_analysis_20261011');
fs.mkdirSync(out,{recursive:true});
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const validatorPath=fs.existsSync(path.join(root,'06_复现脚本/check_recording.mjs'))?path.join(root,'06_复现脚本/check_recording.mjs'):path.join(root,'e5_formal_A_hw6_host_test/check_recording.mjs');
const validator=fs.readFileSync(validatorPath,'utf8').replace("import fs from 'node:fs';import crypto from 'node:crypto';import assert from 'node:assert/strict';",'');
const audit=[],selected=[];
const picks={
 'A_BASE_1':'E5F_A_BASE_HW6_L1200_R1200_M0g_S2mm_run01_full.csv',
 'A_BASE_2':'E5F_A_BASE_HW6_L1200_R1200_M0g_S2mm_run02.csv',
 'A_BASE_3':'E5F_A_BASE_HW6_L1200_R1200_M0g_S2mm_run03.csv',
 'A_BLNN_1':'E5F_A_BLNN_HW6_L1200_R1200_M0g_S2mm_run01_full.csv',
 'A_BLNN_2':'E5F_A_BLNN_HW6_L1200_R1200_M0g_S2mm_run02.csv',
 'A_BLNN_3':'E5F_A_BLNN_HW6_L1200_R1200_M0g_S2mm_run03_attempt02.csv',
 'A_CA_BLNN_1':'5F_A_CA_BLNN_HW6_L1200_R1200_M0g_S2mm_run01.csv',
 'A_CA_BLNN_2':'E5F_A_CA_BLNN_HW6_L1200_R1200_M0g_S2mm_run02.csv',
 'A_CA_BLNN_3':'E5F_A_CA_BLNN_HW6_L1200_R1200_M0g_S2mm_run03_attempt02_full.csv'
};
const stats=x=>({mean:x.reduce((a,v)=>a+v,0)/x.length,sd:x.length>1?Math.sqrt(x.reduce((s,v)=>s+(v-x.reduce((a,z)=>a+z,0)/x.length)**2,0)/(x.length-1)):null,min:Math.min(...x),max:Math.max(...x)});
const rms=x=>Math.sqrt(x.reduce((s,v)=>s+v*v,0)/x.length),peak=x=>Math.max(...x.map(Math.abs));
for(const condition of ['A','B']) {
 const limit=condition==='A'?1200:800,buildBase=condition==='A'?61000:63000;
 const raw=fs.existsSync(path.join(root,'01_原始数据',condition))?path.join(root,'01_原始数据',condition):path.join(root,'experiment5_setup/01_原始数据',condition==='A'?'正式A_无载2mm_1200':'正式B_无载2mm_L1200_R800');
 let body=validator;
 if(condition==='B')for(const [a,b] of [['[[61010,10],[61011,11],[61012,12]]','[[63010,10],[63011,11],[63012,12]]'],['r[19]===1200&&r[20]===1200','r[19]===1200&&r[20]===800'],['Math.abs(r[13+ch])>1200','Math.abs(r[13+ch])>r[19+ch]'],['Math.trunc(1200*(30-speed)/20)','Math.trunc(r[19+ch]*(30-speed)/20)']]){assert.equal(body.split(a).length-1,1);body=body.replace(a,b);}
 const check=(file,bytes)=>{let result;try{vm.runInNewContext(body,{fs:{readFileSync:()=>bytes},crypto,assert,process:{argv:['node','check',file]},console:{log:s=>result=JSON.parse(s)}},{timeout:10000});return result;}catch(e){return {eligibleForNumericalComparison:false,error:e.message};}};
 for(const filename of fs.readdirSync(raw).filter(f=>f.endsWith('.csv')).sort()) {
  const file=path.join(raw,filename),bytes=fs.readFileSync(file),lines=bytes.toString('utf8').replace(/^\uFEFF/,'').trim().split(/\r?\n/);
  assert.deepEqual(lines[0].split(','),Array.from({length:107},(_,i)=>'I'+i));
  const rows=lines.slice(1).map(l=>l.split(',').map(Number));assert.ok(rows.every((r,i)=>r.length===107&&r.every(Number.isFinite)&&lines[i+1].split(',').every(v=>v.trim()!=='')));
  const e={condition,filename,file,sha256:hash(bytes),modifiedTime:fs.statSync(file).mtime.toISOString(),rows:rows.length,wholeFile:check(file,bytes),unidentifiedRows:[],runs:[]};
  const known=r=>[10,11,12].includes(r[21])&&r[63]===buildBase+r[21]&&r[87]===6&&r[19]===1200&&r[20]===limit&&r[104]===0;
  let start=null;
  const finish=end=>{
   const rs=rows.slice(start,end+1),segment=Buffer.from([lines[0],...lines.slice(start+1,end+2)].join('\n')+'\n');
   const s=check(file,segment),method=rs[0][21],name=['BASE','BLNN','CA_BLNN'][method-10];
   e.runs.push({method,name,csvLines:[start+2,end+2],segmentSHA256:hash(segment),summary:s});
   const match=filename.match(/run(\d+)/),run=match?+match[1]:null,id=`${condition}_${name}_${run}`;
   const expected=condition==='A'?picks[id]:`E5F_B_${name}_HW6_L1200_R800_M0g_S2mm_run${String(run).padStart(2,'0')}.csv`;
   if(filename===expected&&s.eligibleForNumericalComparison) {
    assert.deepEqual(s.phaseChanges.map(p=>p.phase),[0,1,2,3,4,5,6]);assert.equal(s.motionRows,400);
    const motion=rs.filter(r=>r[2]>=2&&r[2]<=5),active=rs.filter(r=>r[106]===1),last=rs.at(-1);
    const left=motion.map(r=>1000*(r[5]-r[3])),right=motion.map(r=>1000*(r[6]-r[3])),sync=motion.map(r=>1000*(r[5]-r[6]));
    const percentile=(x,p)=>{x=x.slice().sort((a,b)=>a-b);const i=(x.length-1)*p,k=Math.floor(i);return x[k]+(x[Math.ceil(i)]-x[k])*(i-k);};
    const vals={trackingRMSE_mm:Math.sqrt((rms(left)**2+rms(right)**2)/2),leftRMSE_mm:rms(left),rightRMSE_mm:rms(right),syncRMS_mm:rms(sync),peakTracking_mm:s.peakTracking_mm,peakSync_mm:s.peakSync_mm,
     terminalMean_mm:1000*(last[5]+last[6])/2,terminalMaxAbs_mm:1000*Math.max(Math.abs(last[5]),Math.abs(last[6])),terminalSyncAbs_mm:1000*Math.abs(last[5]-last[6]),postStop_s:s.postPhase6_seconds,
     leftCmdRMS_counts:rms(motion.map(r=>r[13])),rightCmdRMS_counts:rms(motion.map(r=>r[14])),leftCmdPeak_counts:peak(motion.map(r=>r[13])),rightCmdPeak_counts:peak(motion.map(r=>r[14])),
     sampledLeftBoundary_pct:100*motion.filter(r=>Math.abs(r[13])>=r[19]-1).length/motion.length,sampledRightBoundary_pct:100*motion.filter(r=>Math.abs(r[14])>=r[20]-1).length/motion.length,
     fullActivityLeftBoundaryCount:last[61],fullActivityRightBoundaryCount:last[62],growthCount:last[74],leftProjectionCount:last[75],rightProjectionCount:last[76],withdrawalCount:last[77],contractionCount:last[105],
     leftNodes:last[35],rightNodes:last[36],leftWeightNormPeak:peak(active.map(r=>r[37])),rightWeightNormPeak:peak(active.map(r=>r[38])),sampledLearningMean:stats(active.map(r=>r[32])).mean,sampledLearningZero_pct:100*active.filter(r=>r[32]===0).length/active.length,
     coreSampleMean_us:stats(active.map(r=>r[41])).mean,coreSampleP95_us:percentile(active.map(r=>r[41]),.95),corePeak_us:s.corePeak_us,wholePeak_us:s.wholePeak_us,batteryMin_V:s.battery_V[0],batteryMax_V:s.battery_V[1],leftTempMax_C:Math.max(...rs.map(r=>r[29])),rightTempMax_C:Math.max(...rs.map(r=>r[30]))};
    selected.push({id,condition,name,method,run,filename,source:file,sourceSHA256:e.sha256,csvLines:[start+2,end+2],segmentSHA256:hash(segment),build:buildBase+method,limits:[1200,limit],metrics:vals,trajectory:motion.map((r,i)=>({sample:i,t_s:r[79],phase:r[2],reference_mm:r[3]*1000,left_mm:r[5]*1000,right_mm:r[6]*1000,leftError_mm:left[i],rightError_mm:right[i],meanError_mm:(left[i]+right[i])/2,sync_mm:sync[i],leftCmd_counts:r[13],rightCmd_counts:r[14],learning:r[32],gate:r[31],mask:r[68],deltaLeft_counts:r[66],deltaRight_counts:r[67],leftNodes:r[35],rightNodes:r[36]}))});
   }
   start=null;
  };
  for(let i=0;i<rows.length;i++) {
   const r=rows[i],p=rows[i-1];
   if(!known(r)){if(start!==null)finish(i-1);e.unidentifiedRows.push({csvLine:i+2,build:r[63],method:r[21],profile:r[87],phase:r[2],uptime:r[0]});continue;}
   if(start!==null&&(r[63]!==p[63]||r[0]<=p[0]||r[1]<=p[1]))finish(i-1);
   if(start===null)start=i;
  }
  if(start!==null)finish(rows.length-1);assert.equal(hash(fs.readFileSync(file)),e.sha256);audit.push(e);
 }
}
assert.equal(new Set(selected.map(s=>s.id)).size,selected.length,'Duplicate selected slot');
selected.sort((a,b)=>a.condition.localeCompare(b.condition)||a.method-b.method||a.run-b.run);
assert.equal(selected.length,18,'Do not finalize an incomplete comparison');
for(const s of selected)assert.ok(s.trajectory.every((p,i)=>Math.abs(p.t_s-(4+i*.04))<2e-5),'Unequal action-time grid; do not interpolate silently');
const exportRelations=[];
for(const [a,b] of [
 ['E5F_A_BASE_HW6_L1200_R1200_M0g_S2mm_run01.csv','E5F_A_BASE_HW6_L1200_R1200_M0g_S2mm_run01_full.csv'],
 ['E5F_A_BLNN_HW6_L1200_R1200_M0g_S2mm_run01.csv','E5F_A_BLNN_HW6_L1200_R1200_M0g_S2mm_run01_full.csv'],
 ['E5F_A_CA_BLNN_HW6_L1200_R1200_M0g_S2mm_run03_attempt02.csv','E5F_A_CA_BLNN_HW6_L1200_R1200_M0g_S2mm_run03_attempt02_full.csv']]) {
 const ea=audit.find(e=>e.filename===a),eb=audit.find(e=>e.filename===b);
 const la=fs.readFileSync(ea.file,'utf8').trim().split(/\r?\n/).slice(1),lb=fs.readFileSync(eb.file,'utf8').trim().split(/\r?\n/).slice(1);
 const map=new Map(lb.map(l=>{const r=l.split(',');return [r[1],l];}));
 let common=0,identical=0,numericallyIdentical=0;for(const l of la){const r=l.split(',');if(map.has(r[1])){common++;const other=map.get(r[1]);if(other===l)identical++;if(r.map(Number).every((v,i)=>v===Number(other.split(',')[i])))numericallyIdentical++;}}
 exportRelations.push({original:a,extended:b,commonSequenceRows:common,identicalRows:identical,numericallyIdenticalRows:numericallyIdentical,allOriginalRowsPresentUnchanged:identical===la.length,interpretation:numericallyIdentical===la.length?'All original numeric rows preserved in extended file; count once.':'Numeric differences exist at common sequence IDs; filename alone does not establish same-history reexport.'});
}
const summaries=[];
for(const condition of ['A','B'])for(const name of ['BASE','BLNN','CA_BLNN']){
 const trials=selected.filter(s=>s.condition===condition&&s.name===name);if(!trials.length)continue;
 summaries.push({condition,name,n:trials.length,metrics:Object.fromEntries(Object.keys(trials[0].metrics).map(key=>[key,stats(trials.map(t=>t.metrics[key]))]))});
}
fs.writeFileSync(path.join(out,'analysis.json'),JSON.stringify({created:'2026-10-11',scope:'formal A and B only',rawPreserved:true,comparativeWindow:'phases 2-5; 400 frames at nominal 40ms; no interpolation',expectedSlots:18,selectedSlots:selected.length,audit,exportRelations,selected,summaries},null,2));
console.log(JSON.stringify({files:audit.length,selected:selected.length,slots:selected.map(s=>s.id),audit:audit.map(e=>({condition:e.condition,filename:e.filename,unidentifiedRows:e.unidentifiedRows.length,runs:e.runs.map(r=>({method:r.method,lines:r.csvLines,passed:r.summary.eligibleForNumericalComparison,error:r.summary.error,problems:r.summary.problems,post:r.summary.postPhase6_seconds}))})),summaries:summaries.map(s=>({condition:s.condition,name:s.name,n:s.n,track:s.metrics.trackingRMSE_mm,sync:s.metrics.syncRMS_mm}))},null,2));
