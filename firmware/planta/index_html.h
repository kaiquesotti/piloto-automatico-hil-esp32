#pragma once

// Pagina web embutida no ESP32 da planta (HTML/CSS/JS) - MODELO NAO LINEAR.
// Fica em header separado para nao confundir o gerador de prototipos do .ino.

static const char kIndexHtml[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="pt-br"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Piloto automatico - Planta (nao linear)</title>
<style>
*{box-sizing:border-box}
body{font-family:Arial,Helvetica,sans-serif;margin:0;background:#0f1720;color:#e8eef5}
header{background:#174a7e;padding:12px 18px;display:flex;justify-content:space-between;align-items:center;gap:12px}
header h1{margin:0;font-size:17px}
#ip{font-size:12px;color:#cfe2f5}
main{padding:14px;display:grid;grid-template-columns:340px 1fr;gap:14px}
.card{background:#182633;border:1px solid #26384a;border-radius:10px;padding:14px}
h2{font-size:13px;margin:0 0 8px;color:#9fc3e8;text-transform:uppercase;letter-spacing:.5px}
label{display:flex;justify-content:space-between;align-items:center;gap:8px;font-size:13px;margin:5px 0}
input[type=number]{width:104px;background:#0f1720;color:#e8eef5;border:1px solid #34506b;border-radius:6px;padding:4px 6px}
textarea{width:100%;background:#0f1720;color:#e8eef5;border:1px solid #34506b;border-radius:6px;padding:6px;font-family:Consolas,monospace;font-size:12px}
.hint{font-size:11px;color:#7f97ad;margin:-2px 0 6px}
.derived{font-size:12px;color:#9fb3c8;margin-top:8px;line-height:1.6}
.btns{display:flex;gap:8px;margin-top:12px;flex-wrap:wrap}
button{border:0;border-radius:8px;padding:9px 14px;font-size:14px;cursor:pointer;color:#fff}
.go{background:#287a4b}.stop{background:#c0392b}.apply{background:#1e88a8}.clear{background:#6b7280}
.csv{background:#5a4b8a;text-decoration:none;display:inline-block;text-align:center;border-radius:8px;padding:9px 14px;color:#fff;font-size:14px}
.readout{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:10px}
.ro{background:#0f1720;border:1px solid #26384a;border-radius:8px;padding:8px}
.ro b{display:block;font-size:11px;color:#8fb0cf;font-weight:normal}
.ro span{font-size:15px}
canvas{width:100%;background:#0b1219;border-radius:8px;display:block}
.legend{font-size:12px;margin:6px 0 12px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 4px 0 12px}
</style></head><body>
<header><h1>Piloto automatico &mdash; PLANTA nao linear</h1>
<div style="text-align:right"><div id="ip"></div><div id="stat" style="font-size:11px;color:#9fb3c8"></div></div></header>
<main>
 <section class="card">
  <h2>Planta nao linear</h2>
  <label>m (kg)<input id="m" type="number" step="1"></label>
  <label>g (m/s&sup2;)<input id="g" type="number" step="0.01"></label>
  <label>rho<input id="rho" type="number" step="0.001"></label>
  <label>A (m&sup2;)<input id="A" type="number" step="0.01"></label>
  <label>Cd<input id="Cd" type="number" step="0.01"></label>
  <label>uw (m/s)<input id="uw" type="number" step="0.1"></label>
  <label>f<input id="f" type="number" step="0.001"></label>
  <label>u0 inicial (km/h)<input id="speed0" type="number" step="0.1"></label>
  <label>Ts (s)<input id="ts" type="number" step="0.001"></label>

  <h2 style="margin-top:10px">Atuador da borboleta</h2>
  <label>Ka (ganho)<input id="ka" type="number" step="0.1"></label>
  <label>TauA (s)<input id="tauA" type="number" step="0.01"></label>

  <h2 style="margin-top:10px">Ruido de medicao</h2>
  <label>ruido (%)<input id="noise" type="number" step="0.1"></label>
  <div class="hint">0 = sem ruido; sigma em % de |u|.</div>

  <h2 style="margin-top:10px">Controlador PID</h2>
  <label>Kp<input id="kp" type="number" step="0.0001"></label>
  <label>Ki<input id="ki" type="number" step="0.0001"></label>
  <label>Kd<input id="kd" type="number" step="0.0001"></label>
  <label>N (filtro D)<input id="nf" type="number" step="0.0001"></label>

  <h2 style="margin-top:10px">Referencia (degrau)</h2>
  <label>ref inicial (km/h)<input id="refInit" type="number" step="0.1"></label>
  <label>ref final (km/h)<input id="refFinal" type="number" step="0.1"></label>
  <label>ref tempo (s)<input id="refTime" type="number" step="0.1"></label>

  <h2 style="margin-top:10px">Sinal Theta (graus)</h2>
  <div class="hint">Um ponto por linha: tempo,valor. Mantem o valor (degrau/ZOH).<br>
  Ex.: 0,0 &nbsp; 50,40 &nbsp; 100,20 &nbsp; 150,0 &nbsp; 200,0</div>
  <textarea id="theta" rows="7" spellcheck="false"></textarea>

  <h2 style="margin-top:10px">Visualizacao</h2>
  <label>janela (s)<input id="window" type="number" step="1"></label>
  <div class="hint">0 = corrida inteira (com decima&ccedil;&atilde;o); &gt;0 = &uacute;ltimos N s em detalhe total.</div>
  <div class="derived" id="derived"></div>
  <div class="btns">
    <button class="apply" onclick="applyParams()">Aplicar</button>
    <button class="go" onclick="setRun(1)">Iniciar</button>
    <button class="stop" onclick="setRun(0)">Parar</button>
    <button class="clear" onclick="clearAll()">Limpar</button>
    <a class="csv" href="/csv">Baixar CSV</a>
  </div>
 </section>

 <section class="card">
  <div class="readout">
    <div class="ro"><b>Tempo</b><span id="ro_t">0,00 s</span></div>
    <div class="ro"><b>Estado</b><span id="ro_st">parado</span></div>
    <div class="ro"><b>Amostras</b><span id="ro_n">0</span></div>
    <div class="ro"><b>resolucao</b><span id="ro_res">-</span></div>
    <div class="ro"><b>ref r (km/h)</b><span id="ro_r">-</span></div>
    <div class="ro"><b>velocidade u (km/h)</b><span id="ro_u">-</span></div>
    <div class="ro"><b>erro e (km/h)</b><span id="ro_e">-</span></div>
    <div class="ro"><b>Fx (N)</b><span id="ro_fx">-</span></div>
    <div class="ro"><b>Theta (graus)</b><span id="ro_th">-</span></div>
  </div>

  <h2>Velocidade r x u</h2>
  <canvas id="chartV" height="340"></canvas>
  <div class="legend"><span class="dot" style="background:#e8b84b"></span>referencia r
    <span class="dot" style="background:#3fb0e0"></span>velocidade u</div>

  <h2>Forca de controle Fx</h2>
  <canvas id="chartFx" height="160"></canvas>
  <div class="legend"><span class="dot" style="background:#c86bd6"></span>Fx (saida do atuador)</div>

  <h2>Inclinacao Theta</h2>
  <canvas id="chartTh" height="150"></canvas>
  <div class="legend"><span class="dot" style="background:#6fd08a"></span>&theta; (graus)</div>
 </section>
</main>
<script>
var numFields=['m','g','rho','A','Cd','uw','f','ka','tauA','noise','speed0','kp','ki','kd','nf','ts','refInit','refFinal','refTime','window'];
var S={t:[],rK:[],uK:[],fx:[],th:[]};
var curEpoch=-1;
var DPR=Math.max(1,Math.min(2,window.devicePixelRatio||1));
function el(id){return document.getElementById(id);}
function fmt(x,d){return (x===undefined||x===null||isNaN(x))?'-':Number(x).toFixed(d);}
function fitAll(){fitCanvas(el('chartV'),340);fitCanvas(el('chartFx'),160);fitCanvas(el('chartTh'),150);}
function fitCanvas(cv,cssH){var w=cv.parentElement.clientWidth-28;if(w<320)w=320;
  cv.width=Math.round(w*DPR);cv.height=Math.round(cssH*DPR);cv.style.width='100%';cv.style.height=cssH+'px';}
function fill(p){
  numFields.forEach(function(k){if(p[k]!==undefined&&el(k))el(k).value=p[k];});
  if(p.theta!==undefined)el('theta').value=p.theta;
  el('derived').innerHTML='u0 = '+(p.speed0/3.6).toFixed(3)+' m/s<br>'+
    'Ki = '+p.ki.toFixed(4)+'<br>Kd = '+p.kd.toFixed(2)+' , N = '+p.nf.toFixed(4)+'<br>'+
    'Tf deriv = '+ (1/p.nf).toFixed(3)+' s<br>'+
    'Atuador: Ka = '+p.ka.toFixed(2)+' , TauA = '+p.tauA.toFixed(3)+' s<br>'+
    'Ruido = '+p.noise.toFixed(2)+' %';
  el('ip').textContent='http://'+location.hostname+'/';
}
function loadParams(){fetch('/params').then(r=>r.json()).then(fill);}
function applyParams(){var b=new URLSearchParams();
  numFields.forEach(function(k){var e=el(k);if(e&&e.value!=='')b.append(k,e.value);});
  b.append('theta',el('theta').value);
  fetch('/apply',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b.toString()})
   .then(r=>r.json()).then(fill);
}
function clearSeries(){S={t:[],rK:[],uK:[],fx:[],th:[]};el('ro_t').textContent='0,00 s';el('ro_n').textContent='0';
  ['ro_r','ro_u','ro_e','ro_fx','ro_th'].forEach(i=>el(i).textContent='-');drawAll();}
function setRun(s){if(s===1)clearSeries();fetch('/run?state='+s).then(r=>r.json()).then(fill);}
function clearAll(){clearSeries();fetch('/clear').then(r=>r.json()).then(fill);}
function minmax(arrs){var lo=Infinity,hi=-Infinity;
  for(var a=0;a<arrs.length;a++){var v=arrs[a];for(var i=0;i<v.length;i++){var x=v[i];if(x<lo)lo=x;if(x>hi)hi=x;}}
  if(!isFinite(lo)||!isFinite(hi)){lo=0;hi=1;}
  if(hi-lo<1e-9){var c=(hi===0)?1:Math.abs(hi);hi=hi+c*0.1;lo=lo-c*0.1;}
  var pad=(hi-lo)*0.08;return [lo-pad,hi+pad];}
function nice(v){var a=Math.abs(v);return a>=100?v.toFixed(0):(a>=1?v.toFixed(2):(a>=0.01?v.toFixed(3):v.toExponential(1)));}
function drawPlot(cv,xs,arrs,cols,fillUnder){
  var ctx=cv.getContext('2d');var W=cv.width/DPR,H=cv.height/DPR;
  ctx.setTransform(DPR,0,0,DPR,0,0);ctx.clearRect(0,0,W,H);
  var L=64,R=14,T=12,B=26;var n=xs.length;
  var x0=0,x1=(n>0)?xs[n-1]:1;if(x1<=x0)x1=x0+1;
  var mm=minmax(arrs),ylo=mm[0],yhi=mm[1];
  function X(t){return L+(W-L-R)*(t-x0)/(x1-x0);}
  function Y(v){return H-B-(H-T-B)*(v-ylo)/(yhi-ylo);}
  ctx.lineWidth=1;ctx.font='11px Arial';
  for(var k=0;k<=5;k++){var yy=T+(H-T-B)*k/5;
    ctx.strokeStyle='#223342';ctx.beginPath();ctx.moveTo(L,yy);ctx.lineTo(W-R,yy);ctx.stroke();
    ctx.fillStyle='#8fb0cf';ctx.textAlign='right';ctx.fillText(nice(yhi-(yhi-ylo)*k/5),L-6,yy+4);}
  for(var k=0;k<=5;k++){var xx=L+(W-L-R)*k/5;
    ctx.strokeStyle='#1b2836';ctx.beginPath();ctx.moveTo(xx,T);ctx.lineTo(xx,H-B);ctx.stroke();
    ctx.fillStyle='#8fb0cf';ctx.textAlign='center';ctx.fillText((x0+(x1-x0)*k/5).toFixed(1),xx,H-8);}
  ctx.fillStyle='#8fb0cf';ctx.textAlign='center';ctx.fillText('tempo (s)',(L+W-R)/2,H-0.5);
  for(var a=0;a<arrs.length;a++){var v=arrs[a];if(v.length<1)continue;
    if(fillUnder&&a===arrs.length-1){ctx.fillStyle=cols[a]+'22';ctx.beginPath();ctx.moveTo(X(xs[0]),Y(ylo));
      for(var i=0;i<v.length;i++)ctx.lineTo(X(xs[i]),Y(v[i]));ctx.lineTo(X(xs[v.length-1]),Y(ylo));ctx.closePath();ctx.fill();}
    ctx.strokeStyle=cols[a];ctx.lineWidth=2;ctx.lineJoin='round';ctx.lineCap='round';ctx.beginPath();
    for(var i=0;i<v.length;i++){var px=X(xs[i]),py=Y(v[i]);if(i===0)ctx.moveTo(px,py);else ctx.lineTo(px,py);}
    ctx.stroke();
    if(v.length){ctx.fillStyle=cols[a];ctx.beginPath();ctx.arc(X(xs[v.length-1]),Y(v[v.length-1]),3,0,7);ctx.fill();}}
}
function drawAll(){
  drawPlot(el('chartV'),S.t,[S.rK,S.uK],['#e8b84b','#3fb0e0'],true);
  drawPlot(el('chartFx'),S.t,[S.fx],['#c86bd6'],true);
  drawPlot(el('chartTh'),S.t,[S.th],['#6fd08a'],true);
}
function poll(){
 fetch('/data').then(r=>r.arrayBuffer()).then(function(buf){
   var dv=new DataView(buf);if(dv.byteLength<32)return;var o=0;
   var magic=dv.getUint32(o,true);o+=4;if(magic!==0x31444950)return;
   var epoch=dv.getUint32(o,true);o+=4;
   var count=dv.getUint32(o,true);o+=4;
   var stride=dv.getUint32(o,true);o+=4;
   var np=dv.getUint32(o,true);o+=4;
   var ts=dv.getFloat32(o,true);o+=4;
   var simTime=dv.getFloat32(o,true);o+=4;
   var running=dv.getUint8(o);o+=4;
   if(epoch!==curEpoch){curEpoch=epoch;S={t:[],rK:[],uK:[],fx:[],th:[]};}
   var t=new Array(np),rK=new Array(np),uK=new Array(np),fx=new Array(np),th=new Array(np);
   for(var i=0;i<np;i++){t[i]=dv.getFloat32(o,true);o+=4;
     rK[i]=dv.getFloat32(o,true)*3.6;o+=4;
     uK[i]=dv.getFloat32(o,true)*3.6;o+=4;
     fx[i]=dv.getFloat32(o,true);o+=4;
     th[i]=dv.getFloat32(o,true);o+=4;}
   S.t=t;S.rK=rK;S.uK=uK;S.fx=fx;S.th=th;
   el('ro_t').textContent=fmt(simTime,2)+' s';
   el('ro_st').textContent=running?'RODANDO':'parado';
   el('ro_n').textContent=count;
   el('ro_res').textContent=(ts? (ts*1000).toFixed(2)+' ms':'--');
   if(np>0){el('ro_r').textContent=fmt(rK[np-1],2);el('ro_u').textContent=fmt(uK[np-1],2);
     el('ro_e').textContent=fmt(rK[np-1]-uK[np-1],2);el('ro_fx').textContent=fmt(fx[np-1],1);
     el('ro_th').textContent=fmt(th[np-1],3);}
   drawAll();
 });
}
window.addEventListener('resize',function(){fitAll();drawAll();});
var lastUptime=null;
function pollStatus(){
 fetch('/status').then(r=>r.json()).then(function(s){
   var txt='ESP32: '+s.reason+' | '+(s.uptime_ms/1000).toFixed(0)+' s | heap '+s.heap+' B';
   var bad=(s.reason!=='POWERON'&&s.reason!=='SOFTWARE');
   if(lastUptime!==null && s.uptime_ms<lastUptime-1000){txt='ESP32 REINICIOU ('+s.reason+')';bad=true;}
   lastUptime=s.uptime_ms;
   el('stat').textContent=txt;
   el('stat').style.color=bad?'#ffb4a2':'#9fb3c8';
 }).catch(function(){el('stat').textContent='ESP32 sem resposta...';});
}
fitAll();loadParams();setInterval(poll,400);poll();setInterval(pollStatus,3000);pollStatus();
</script></body></html>
)HTML";
