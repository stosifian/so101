import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
f=35.0; px=35.9/7008*1000  # µm
B=20.0  # blur budget µm (~4 px): defocus ⊕ diffraction
lam=0.55
s=np.linspace(220,1000,400)  # mm, MFD 0.22 m
cols={8:"#2a78d6",11:"#eb6834",16:"#1baf7a"}
ink,muted,grid="#1f1f1d","#6b6a63","#e4e3dd"
fig,ax=plt.subplots(1,2,figsize=(13,5.2))
res={}
for N,c in cols.items():
    airy=2.44*lam*N
    cc=np.sqrt(max(B**2-airy**2,0))/1000  # mm
    if cc<=0: dof=np.zeros_like(s)
    else:
        H=f*f/(N*cc)+f
        near=s*(H-f)/(H+s-2*f); far=np.where(H>s, s*(H-f)/(H-s), np.inf)
        dof=far-near
    res[N]=dof
    ax[0].plot(s/1000,dof,color=c,lw=2)
    k=-1 if N!=16 else 250
    lbl=f"f/{N}" + ("  (diffraction alone ≈ budget)" if N==16 else "")
    ax[0].annotate(lbl,(s[k]/1000,dof[k]),xytext=(4,4),textcoords="offset points",color=ink,fontsize=10)
for d in (380,500):
    ax[0].axvline(d/1000,color=muted,lw=0.8,ls=":"); ax[1].axvline(d/1000,color=muted,lw=0.8,ls=":")
ax[0].axvline(0.22,color=muted,lw=0.8); ax[0].text(0.225,ax[0].get_ylim()[1]*0.92 if False else 5,"MFD 0.22 m",color=muted,fontsize=9,rotation=90,va="bottom")
ax[0].set(title=f"Total depth of field (blur ≤ {B:g} µm ≈ {B/px:.0f} px, incl. diffraction)",xlabel="focus distance (m)",ylabel="DoF (mm)",ylim=(0,300),xlim=(0.2,1.0))
W=36.0*(s-f)/f; H_=24.0*(s-f)/f
ax[1].plot(s/1000,W,color="#2a78d6",lw=2); ax[1].annotate("frame width",(s[-1]/1000,W[-1]),xytext=(-70,6),textcoords="offset points",color=ink)
ax[1].plot(s/1000,H_,color="#eb6834",lw=2); ax[1].annotate("frame height",(s[-1]/1000,H_[-1]),xytext=(-75,6),textcoords="offset points",color=ink)
for val,txt in ((227,"iPad screen width 227 mm"),(360,"tour y-extent 360 mm"),(60,"gripper target ~60 mm")):
    ax[1].axhline(val,color=muted,lw=0.8,ls="--"); ax[1].text(0.205,val+6,txt,color=muted,fontsize=9)
ax[1].set(title="Field of view (35 mm on full frame)",xlabel="focus distance (m)",ylabel="mm",xlim=(0.2,1.0),ylim=(0,1100))
for a in ax:
    a.grid(color=grid,lw=0.6); [sp.set_visible(False) for k,sp in a.spines.items() if k in ("top","right")]
    a.tick_params(colors=muted); a.title.set_color(ink); a.xaxis.label.set_color(muted); a.yaxis.label.set_color(muted)
ax[0].text(0.385,285,"0.38 m",color=muted,fontsize=9); ax[0].text(0.505,285,"0.5 m",color=muted,fontsize=9)
fig.tight_layout(); fig.savefig("docs/figures/dof_fov_sel35f18_4px.png",dpi=120)
for d in (300,380,500,700):
    i=np.argmin(abs(s-d)); print(f"{d} mm: DoF f/8 {res[8][i]:.0f}, f/11 {res[11][i]:.0f}, f/16 {res[16][i]:.0f} mm; frame {W[i]:.0f}x{H_[i]:.0f} mm; px {W[i]/7008*1000:.0f} µm")
