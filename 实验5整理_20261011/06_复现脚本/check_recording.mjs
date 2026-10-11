import fs from 'node:fs';import crypto from 'node:crypto';import assert from 'node:assert/strict';
const file=process.argv[2];assert.ok(file,'107-channel formal CSV path required');
const bytes=fs.readFileSync(file),sha=crypto.createHash('sha256').update(bytes).digest('hex');
const lines=bytes.toString('utf8').replace(/^\uFEFF/,'').trim().split(/\r?\n/);
assert.deepEqual(lines.shift().split(','),Array.from({length:107},(_,i)=>`I${i}`),'Not the 107-channel frozen formal schema');
const parsed=lines.map((l,i)=>({line:i+2,raw:l.split(',')}));
assert.ok(parsed.every(r=>r.raw.length===107&&r.raw.every(v=>v.trim()!==''&&Number.isFinite(Number(v)))),'Malformed/nonfinite row: preserve file, not silently clean');
const rows=parsed.map(r=>r.raw.map(Number));assert.ok(rows.length);
const ids=new Map([[61010,10],[61011,11],[61012,12]]),build=rows[0][63],method=ids.get(build);
assert.ok(method,'Unknown formal build');
assert.ok(rows.every(r=>r[63]===build&&r[21]===method&&r[87]===6&&r[19]===1200&&r[20]===1200&&r[104]===0),'Mixed/wrong metadata: not eligible for frozen A');
const motion=rows.filter(r=>r[2]>=2&&r[2]<=5&&r[22]===0),control=rows.filter(r=>r[106]===1),stop=rows.filter(r=>r[2]===6);
const problems=[],warnings=[],faults=[...new Set(rows.map(r=>r[22]).filter(v=>v!==0))];
const changes=[],gaps=[];
for(let i=0;i<rows.length;i++) {
 const r=rows[i],p=rows[i-1];
 if(!p||r[2]!==p[2])changes.push({time:r[0],phase:r[2],fault:r[22]});
 if(p&&(r[0]<=p[0]||r[1]<=p[1]))problems.push({line:i+2,reason:'reset/second startup'});
 if(p&&r[1]-p[1]!==1) {
  const gap={line:i+2,time:r[0],phase:r[2],missingFrames:r[1]-p[1]-1};gaps.push(gap);
  if((r[2]>=2&&r[2]<=5)||(p[2]>=2&&p[2]<=5))problems.push({...gap,reason:'motion sample gap'});
  else warnings.push({...gap,reason:'gap outside moving/settling comparison phases'});
 }
 if(r[106]!==((r[2]>=1&&r[2]<=5&&r[22]===0)?1:0))problems.push({line:i+2,reason:'activity flag inconsistent'});
 if(r[106]===0&&(r[13]!==0||r[14]!==0||r[32]!==0||r[66]!==0||r[67]!==0))problems.push({line:i+2,reason:'inactive command/NN/learning nonzero'});
}
const peak=(rs,c)=>rs.length?Math.max(...rs.map(r=>Math.abs(r[c]))):null;
for(const r of control) {
 for(let ch=0;ch<2;ch++) {
  if(Math.abs(r[13+ch])>1200||r[13+ch]!==Math.trunc(r[13+ch]))problems.push({time:r[0],reason:'hard amplitude/integer violation'});
  if(Math.abs(r[13+ch]-r[64+ch]-r[66+ch])>1e-6)problems.push({time:r[0],reason:'final command/reference/delta mismatch'});
  const rpm=r[80+ch],speed=Math.abs(rpm),aligned=rpm>0?r[13+ch]:-r[13+ch];
  if((speed>30&&aligned>-Math.min(200,10*(speed-30)))||(speed>10&&speed<=30&&aligned>Math.trunc(1200*(30-speed)/20)))problems.push({time:r[0],reason:'speed envelope violation'});
 }
 if(method===10&&(r[35]!==0||r[36]!==0||r[32]!==0||r[66]!==0||r[67]!==0))problems.push({time:r[0],reason:'BASE NN enabled'});
 if(method===11&&r[32]!==1)problems.push({time:r[0],reason:'ordinary BLNN learning not 1'});
 if(method===12) {
  if(Math.abs(r[66])>300||Math.abs(r[67])>300||(r[68]!==0&&r[32]!==0))problems.push({time:r[0],reason:'CA amplitude/gated learning violation'});
  const k=.006197916666666667,forces=[k*(r[66]-r[67]),-.3*k*(r[66]+r[67])];
  for(let ch=0;ch<2;ch++) {
   const sigma=r[39+ch],budget=ch?6.25:10;
   const excess=Math.min(...[sigma-.5e-6,sigma+.5e-6].map(s=>s*forces[ch]-budget*s*s));
   if(excess>1e-9)problems.push({time:r[0],reason:'CA confirmed sector excess with CSV rounding accounted'});
  }
 }
}
for(const [phase,count] of [[2,100],[3,50],[4,100],[5,150]])if(motion.filter(r=>r[2]===phase).length!==count)problems.push({phase,reason:'incomplete motion/settling phase',expected:count});
if(faults.length)problems.push({reason:'control fault: retain as outcome',faults});
const post=stop.length?rows.at(-1)[0]-stop[0][0]:0,last=rows.at(-1);
if(post<5-1e-5)problems.push({reason:'post-phase6 shorter than 5 seconds',seconds:post});
if(last[2]!==6||last[22]!==0||Math.abs(last[5])>.0005||Math.abs(last[6])>.0005||Math.abs(last[5]-last[6])>.0005||last[80]!==0||last[81]!==0||last[13]!==0||last[14]!==0)problems.push({reason:'final phase/return/stopped state not accepted'});
for(const [col,label] of [[28,'CAN rejected'],[57,'overruns'],[59,'UART errors'],[70,'DMA skipped'],[72,'BMS errors']])if(peak(rows,col)>0)problems.push({reason:label,value:peak(rows,col)});
const summary={file,sha256:sha,build,method,rows:rows.length,phaseChanges:changes,faults,postPhase6_seconds:post,
 motionRows:motion.length,peakTracking_mm:motion.length?Math.max(...motion.map(r=>Math.max(Math.abs(r[5]-r[3]),Math.abs(r[6]-r[3]))))*1000:null,
 peakSync_mm:motion.length?Math.max(...motion.map(r=>Math.abs(r[5]-r[6])))*1000:null,
 final:{positions_mm:last.slice(5,7).map(v=>v*1000),sync_mm:(last[5]-last[6])*1000,rpm:last.slice(80,82),commands:last.slice(13,15)},
 cumulativePeakRPM:[peak(rows,82),peak(rows,83)],corePeak_us:peak(rows,42),wholePeak_us:peak(rows,60),
 battery_V:[Math.min(...rows.map(r=>r[27])),Math.max(...rows.map(r=>r[27]))],contractions:last[105],withdrawals:last[77],
 eligibleForNumericalComparison:problems.length===0,problems,warnings,sequenceGaps:gaps,
 caveats:['Field safety/actual load/voltage condition still require operator confirmation.','Passing one record does not prove method superiority or all-condition reliability.','40ms sampled invariants do not prove every unsampled 1ms value.','Failed attempts are retained as experimental outcomes; no automated deletion, retry or flash.']};
console.log(JSON.stringify(summary,null,2));assert.equal(crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex'),sha);
