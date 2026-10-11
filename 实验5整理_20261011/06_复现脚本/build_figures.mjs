import fs from 'node:fs/promises';import path from 'node:path';import {fileURLToPath} from 'node:url';import sharp from 'sharp';
const dir=process.argv[2]?path.resolve(process.argv[2]):path.dirname(fileURLToPath(import.meta.url)),data=JSON.parse(await fs.readFile(path.join(dir,'analysis.json'),'utf8'));
const dest=path.join(dir,'论文图');await fs.mkdir(dest,{recursive:true});
const names=['BASE','BLNN','CA_BLNN'],colors=['#31688e','#2a9d61','#d57a18'],W=1040,H=760;
const mean=x=>x.reduce((s,v)=>s+v,0)/x.length,sd=x=>Math.sqrt(x.reduce((s,v)=>s+(v-mean(x))**2,0)/(x.length-1));
const esc=t=>String(t).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;');
let content='';
const text=(x,y,t,size=15,anchor='start',extra='')=>`<text x="${x}" y="${y}" font-size="${size}" text-anchor="${anchor}" ${extra}>${esc(t)}</text>`;
const line=(x1,y1,x2,y2,color='#202020',width=1,extra='')=>`<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${color}" stroke-width="${width}" ${extra}/>`;
function start(title,subtitle){content=`<rect width="${W}" height="${H}" fill="white"/>`+text(24,30,title,22)+text(24,54,subtitle,14);names.forEach((n,i)=>{content+=line(670+i*122,31,696+i*122,31,colors[i],3)+text(702+i*122,36,n,13);});}
function panel(index,title,ylab,min,max,xmin=4,xmax=20){const col=index%2,row=Math.floor(index/2),x=col*512+82,y=row*320+109,w=404,h=229;
 const sx=v=>x+(v-xmin)/(xmax-xmin)*w,sy=v=>y+h-(v-min)/(max-min)*h;
 content+=text(x,y-16,title,16);
 for(let i=0;i<=4;i++){const val=min+(max-min)*i/4,yy=sy(val);content+=line(x,yy,x+w,yy,'#e7e7e7')+text(x-10,yy+5,Math.abs(val)<1e-12?'0':Number(val.toFixed(3)).toString(),13,'end');}
 content+=line(x,y,x,y+h)+line(x,y+h,x+w,y+h);
 for(let i=0;i<=4;i++){const val=xmin+(xmax-xmin)*i/4,xx=sx(val);content+=line(xx,y+h,xx,y+h+5)+text(xx,y+h+23,Number(val.toFixed(2)),13,'middle');}
 content+=text(x+w/2,y+h+47,'Action time (s)',14,'middle')+`<text transform="translate(${x-58},${y+h/2}) rotate(-90)" font-size="14" text-anchor="middle">${esc(ylab)}</text>`;
 return {x,y,w,h,sx,sy};
}
function series(p,points,color,width=1.8,opacity=1,dash=''){content+=`<polyline points="${points.map(([x,y])=>`${p.sx(x).toFixed(2)},${p.sy(y).toFixed(2)}`).join(' ')}" fill="none" stroke="${color}" stroke-width="${width}" opacity="${opacity}" ${dash?'stroke-dasharray="'+dash+'"':''}/>`;}
function curves(p,condition,key){names.forEach((name,j)=>{const trials=data.selected.filter(t=>t.condition===condition&&t.name===name);trials.forEach(t=>series(p,t.trajectory.map(r=>[r.t_s,r[key]]),colors[j],.8,.24));const avg=trials[0].trajectory.map((r,i)=>[r.t_s,mean(trials.map(t=>t.trajectory[i][key]))]);series(p,avg,colors[j],2,1,j===1?'6 3':j===2?'2 2':'');});}
async function save(name){const svg=`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><g font-family="Arial, sans-serif" fill="#202020">${content}</g></svg>`;await fs.writeFile(path.join(dest,name+'.svg'),svg);await sharp(Buffer.from(svg),{density:240}).png().toFile(path.join(dest,name+'.png'));}
start('Formal hardware comparison','Each dot is one run. Mean and sample SD; n = 3 per method and condition.');
for(const [index,condition,key,label,max] of [[0,'A','trackingRMSE_mm','Tracking RMSE (mm)',.4],[1,'B','trackingRMSE_mm','Tracking RMSE (mm)',.4],[2,'A','syncRMS_mm','Synchronization RMS (mm)',.22],[3,'B','syncRMS_mm','Synchronization RMS (mm)',.22]]){
 const p=panel(index,`${condition}: L1200 / R${condition==='A'?1200:800}`,label,0,max,0,4);
 // Replace numeric categorical labels and action-time axis label with method names.
 content=content.replace(text(p.x+p.w/2,p.y+p.h+47,'Action time (s)',14,'middle'),'');
 for(let i=0;i<=4;i++)content=content.replace(text(p.sx(i),p.y+p.h+23,i,13,'middle'),'');
 names.forEach((name,j)=>{const ts=data.selected.filter(s=>s.condition===condition&&s.name===name),values=ts.map(t=>t.metrics[key]),m=mean(values),s=sd(values),xx=p.sx(j+1);content+=text(xx,p.y+p.h+25,name,13,'middle');content+=line(xx,p.sy(m-s),xx,p.sy(m+s),colors[j],2)+line(xx-6,p.sy(m-s),xx+6,p.sy(m-s),colors[j],2)+line(xx-6,p.sy(m+s),xx+6,p.sy(m+s),colors[j],2);values.forEach((v,i)=>{content+=`<circle cx="${xx+(i-1)*9}" cy="${p.sy(v)}" r="4" fill="${colors[j]}" opacity="0.8"/>`;});content+=`<rect x="${xx-4}" y="${p.sy(m)-4}" width="8" height="8" fill="white" stroke="${colors[j]}" stroke-width="2"/>`;});
}
await save('Fig01_individual_metrics');
start('Tracking and synchronization errors','Thick lines: three-run means. Thin lines: all individual runs. No smoothing or interpolation.');
for(const [i,c,k,ylabel] of [[0,'A','meanError_mm','Mean-axis tracking error (mm)'],[1,'B','meanError_mm','Mean-axis tracking error (mm)'],[2,'A','sync_mm','Left - right position (mm)'],[3,'B','sync_mm','Left - right position (mm)']]){const p=panel(i,c+' condition',ylabel,-.7,.7);curves(p,c,k);}
await save('Fig02_error_time_series');
start('Final actuator command comparison','Dashed horizontal lines: configured bounds. Thick means and thin individual runs; counts are not amperes.');
for(const [i,c,k,label,limit] of [[0,'A','leftCmd_counts','Left command (counts)',1200],[1,'B','leftCmd_counts','Left command (counts)',1200],[2,'A','rightCmd_counts','Right command (counts)',1200],[3,'B','rightCmd_counts','Right command (counts)',800]]){const p=panel(i,c+' condition',label,-1400,1400);for(const b of [-limit,limit])content+=line(p.x,p.sy(b),p.x+p.w,p.sy(b),'#777777',1,'stroke-dasharray="5 4"');curves(p,c,k);}
await save('Fig03_final_commands');
start('Learning and node-growth observations','Phases 2-5. These trajectories demonstrate mechanism activity, not comparative superiority.');
for(const [i,c,k,label,min,max] of [[0,'A','learning','Learning coefficient',0,1.1],[1,'B','learning','Learning coefficient',0,1.1],[2,'A','rightNodes','Right-network nodes',0,26],[3,'B','rightNodes','Right-network nodes',0,26]]){const p=panel(i,c+' condition',label,min,max);curves(p,c,k);}
await save('Fig04_learning_and_nodes');
await fs.writeFile(path.join(dir,'figure_manifest.json'),JSON.stringify({figures:['Fig01_individual_metrics','Fig02_error_time_series','Fig03_final_commands','Fig04_learning_and_nodes'],window:'phases 2-5, nominal 40ms snapshots',allRunsShown:true,reference:'analysis.json selected[].trajectory'},null,2));
console.log('Created 4 scientific figure sets, SVG and high-resolution PNG; all 18 runs shown.');
