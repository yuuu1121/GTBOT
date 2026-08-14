"""시뮬 vs 실물 판 클러스터 대조."""
import glob, os, sqlite3
import numpy as np
import matplotlib; matplotlib.use('Agg')
matplotlib.rcParams['font.family']='UnDotum'; matplotlib.rcParams['axes.unicode_minus']=False
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from scipy.cluster.hierarchy import fcluster, linkage

def bag_stats(db):
    con=sqlite3.connect(db)
    tid=con.execute("SELECT id FROM topics WHERE name='/ouster_cluster/box_points'").fetchone()[0]
    rows=con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,)).fetchall()
    n,L,A,R,I,empty=[],[],[],[],[],0
    for (blob,) in rows:
        a=np.array([list(p) for p in pc2.read_points(deserialize_message(blob,PointCloud2),
                     field_names=('x','y','z','intensity'),skip_nans=True)])
        if len(a)==0: empty+=1; continue
        lab=fcluster(linkage(a[:,:2],'single'),0.25,'distance') if len(a)>=2 else np.array([1])
        for k in np.unique(lab):
            q=a[lab==k]
            if len(q)<5: continue
            c=q[:,:2].mean(0); d=q[:,:2]-c
            _,s,vt=np.linalg.svd(d,full_matrices=False)
            n.append(len(q)); L.append(np.ptp(d@vt[0])*100)
            A.append(s[1]/max(s[0],1e-9)); R.append(np.linalg.norm(c)); I.append(q[:,3])
    return dict(n=np.array(n),L=np.array(L),A=np.array(A),R=np.array(R),
                I=np.concatenate(I) if I else np.array([]),empty=empty,total=len(rows))

BAGS=[('정지','lidar_cluster_bag'),('이동 1','lidar_cluster_moviing_bag'),('이동 2','lidar_cluster_moviing2_bag')]
S={lab: bag_stats(f'results/real_lidar_data/{b}/{b}_0.db3') for lab,b in BAGS}
SIM=dict(n=176., L=14.0, A=0.24, R=0.87)      # 시뮬 실측(2026-08-14 측정)

fig,ax=plt.subplots(1,4,figsize=(19,4.4))
labs=[l for l,_ in BAGS]
a=ax[0]
a.bar(range(3),[np.median(S[l]['n']) for l in labs],color='#1f77b4',width=.5,
      yerr=[[np.median(S[l]['n'])-np.percentile(S[l]['n'],5) for l in labs],
            [np.percentile(S[l]['n'],95)-np.median(S[l]['n']) for l in labs]],capsize=5)
a.axhline(SIM['n'],ls='--',c='r',label=f"시뮬 {SIM['n']:.0f}pt @0.87m")
a.set_xticks(range(3)); a.set_xticklabels(labs); a.set_ylabel('클러스터당 점 수')
a.set_title('점 수 — 실물이 시뮬과 같은 자릿수',fontsize=11); a.legend(fontsize=8); a.grid(alpha=.3,axis='y')

a=ax[1]
for l,c in zip(labs,['#1f77b4','#2ca02c','#ff7f0e']):
    a.hist(S[l]['L'],bins=40,range=(8,20),histtype='step',color=c,lw=1.4,label=f'{l} (중앙 {np.median(S[l]["L"]):.1f})')
a.axvline(14.0,ls='--',c='r',label='실제 판 14 cm')
a.set_xlabel('장축 [cm]'); a.set_ylabel('프레임'); a.legend(fontsize=8)
a.set_title('장축 — 실측 치수를 복원한다',fontsize=11); a.grid(alpha=.3)

a=ax[2]
for l,c in zip(labs,['#1f77b4','#2ca02c','#ff7f0e']):
    a.hist(S[l]['A'],bins=40,range=(0,.4),histtype='step',color=c,lw=1.4,label=f'{l} (중앙 {np.median(S[l]["A"]):.3f})')
a.axvline(SIM['A'],ls='--',c='r',label=f"시뮬 {SIM['A']:.2f}")
a.set_xlabel('종횡비(단축/장축)'); a.legend(fontsize=8)
a.set_title('실물이 시뮬보다 훨씬 얇다',fontsize=11); a.grid(alpha=.3)

a=ax[3]
for l,c in zip(labs,['#1f77b4','#2ca02c','#ff7f0e']):
    a.hist(S[l]['I'],bins=50,range=(0,60),histtype='step',color=c,lw=1.4,density=True,
           label=f'{l} (중앙 {np.median(S[l]["I"]):.1f})')
a.set_xlabel('intensity'); a.set_ylabel('밀도'); a.legend(fontsize=8)
a.set_title('실물 강도는 연속값 — 시뮬은 이진 255',fontsize=11); a.grid(alpha=.3)

fig.suptitle('실물 LiDAR 판 검출 vs 시뮬 — Ouster OS0, /ouster_cluster/box_points',fontsize=13)
fig.tight_layout(rect=[0,0,1,.93]); fig.savefig('results/figs/real_vs_sim_plate.png',dpi=110)
print('saved results/figs/real_vs_sim_plate.png\n')
print(f'{"bag":<8}{"프레임":>7}{"미검출":>9}{"점수중앙":>9}{"장축":>8}{"종횡비":>8}{"거리":>8}{"강도중앙":>9}')
for l in labs:
    s=S[l]
    print(f'{l:<8}{s["total"]:>7}{100*s["empty"]/s["total"]:>8.1f}%{np.median(s["n"]):>9.0f}'
          f'{np.median(s["L"]):>7.1f}cm{np.median(s["A"]):>8.3f}{np.median(s["R"]):>7.2f}m{np.median(s["I"]):>9.1f}')
print(f'{"시뮬":<8}{"-":>7}{"0.0%":>9}{SIM["n"]:>9.0f}{SIM["L"]:>7.1f}cm{SIM["A"]:>8.3f}{SIM["R"]:>7.2f}m{"255(이진)":>9}')
