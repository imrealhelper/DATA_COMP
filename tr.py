
import time, random, math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import cvxpy as cp
from scipy.linalg import block_diag

mpl.rcParams.update({
    "figure.dpi": 120,
    "font.family": "serif",
    "axes.grid": True,
    "grid.alpha": .25,
    "xtick.direction": "in",
    "ytick.direction": "in",
})

# ════════════════════════════════════════════════════════════════════════════
class TrajectoryReconPipeline:
    """End‑to‑end pipeline: flight → corruption → ADMM recon → error eval"""

    # ─────────────────── 초기화 ────────────────────────────────────────────
    def __init__(self,
                 m=120.0, S=np.pi*0.12**2,
                 Cd0=0.30, ki=3.0, Cla=6.0,
                 g0=9.80665, dt=0.1, tf=400.0, seed=3001):
        self.m, self.S = m, S
        self.Cd0, self.ki, self.Cla = Cd0, ki, Cla
        self.g0, self.dt, self.tf = g0, dt, tf
        self.Rad2Deg = 180/np.pi; self.Deg2Rad = 1/self.Rad2Deg
        np.random.seed(seed); random.seed(seed)

    # ────────────────────── 1) 발사 / 목표 생성 ────────────────────────────
    def make_launch_points(self, N=100):
        origin_lat, origin_lon = 37.44926561348997, 126.65246235937565
        lat38, lon_min, lon_max = 38.0, 124.5, 130.25
        lons = np.random.uniform(lon_min, lon_max, N)
        lats = np.full_like(lons, lat38)
        m_lat = 111_132.92
        m_lon = m_lat * np.cos(np.deg2rad(origin_lat))
        x = (lons - origin_lon) * m_lon
        y = (lats - origin_lat) * m_lat
        return np.column_stack([x, y, np.zeros_like(x)])

    def make_targets(self, M=500):
        th = np.random.uniform(-np.pi/4, np.pi/4, M)
        d  = np.random.uniform(55e3, 70e3,   M)
        tx, ty = d*np.sin(th), -d*np.cos(th)
        return np.column_stack([tx, ty, np.zeros_like(tx)]), th

    # ────────────────────── 2) 동역학 & 유도 ──────────────────────────────
    def _state_derivative(self, z, v_w, u):
        V, gamma, psi, pn, pe, h = z
        rho = 1.225*np.exp(-h/8500)
        g = self.g0*(6378137/(6378137+h))**2

        c_gam, s_gam = np.cos(gamma), np.sin(gamma)
        c_psi, s_psi = np.cos(psi),   np.sin(psi)
        vn = V*c_gam*c_psi
        ve = V*c_gam*s_psi
        vd = -V*s_gam
        v_ned = np.array([vn, ve, vd])
        v_rel = v_ned - v_w
        v_mag = np.sqrt(v_rel.dot(v_rel))
        Cd = self.Cd0 + self.ki * (4 * self.m**2 / (rho**2 * self.S**2 * self.Cla**4)) * (u[0]**2 + u[1]**2) / v_mag**4
        D_ned = -0.5*rho*self.S*Cd*v_mag*v_rel
        C_ned_to_xyz = np.array([[c_gam*c_psi,  c_gam*s_psi, -s_gam],
                                [     -s_psi,        c_psi,      0],
                                [s_gam*c_psi,  s_gam*s_psi,  c_gam]])
        D_xyz = C_ned_to_xyz.dot(D_ned)
        V_dot    =  D_xyz[0]/self.m   - g*s_gam
        gamma_dot = -D_xyz[2]/(self.m*V) - g*c_gam/V -u[0]/V
        psi_dot   =  D_xyz[1]/(self.m*V) + u[1]/V
        pn_dot = vn
        pe_dot = ve
        h_dot = -vd
        return np.array([V_dot, gamma_dot, psi_dot, pn_dot, pe_dot, h_dot])

    def compute_guidance_cmd(self,t,state,p_target,v_target):

        V, gamma, psi, pn, pe, h = state
        g_bar = 9.8
        u_max = 10
        t_start = 20
        t_cutoff = 1
        if t < t_start:
            return np.array([0,0])
        else:
            p_m = np.array([pn, pe, -h])
            p_r = p_target - p_m
            rho = 1.225*(1-2.256e-5*h)**5.256
            c_gam, s_gam = np.cos(gamma), np.sin(gamma)
            c_psi, s_psi = np.cos(psi),   np.sin(psi)
            vn = V*c_gam*c_psi
            ve = V*c_gam*s_psi
            vd = -V*s_gam
            v_m= np.array([vn, ve, vd])
            v_r =  v_target - v_m
            Omega = np.cross(p_r, v_r) / np.linalg.norm(p_r)**2
            a_cmd = 3*np.cross(v_r, Omega)
            t_go = np.linalg.norm(p_r) / np.linalg.norm(v_r)
            if t_go < t_cutoff:
                a_cmd = np.zeros_like(a_cmd)
            e_sp_y = np.cross(np.array([0,0,1]),v_m)
            e_y = e_sp_y / np.linalg.norm(e_sp_y)
            e_sp_z = np.cross(v_m,e_y)
            e_z = e_sp_z/ np.linalg.norm(e_sp_z)
            u_z = np.dot(e_z,a_cmd) - g_bar*c_gam
            u_y = np.dot(e_y,a_cmd)
            u = np.array([u_z,u_y])
            if np.linalg.norm(u) > u_max:
                u = u * u_max / np.linalg.norm(u)
            return u



    def _guidance_cmd(self, t, state, p_t, v_t=np.zeros(3)):
        g_bar, u_max, t_start, t_cut = 9.8, 10, 20, 1
        if t < t_start: return np.zeros(2)
        V, gam, psi, pn, pe, h = state
        p_now = np.array([pn, pe, -h]); p_rel = p_t - p_now
        cg, sg, cp, sp = np.cos(gam), np.sin(gam), np.cos(psi), np.sin(psi)
        v_now = V*np.array([cg*cp, cg*sp, -sg]); v_rel = v_t - v_now
        t_go  = np.linalg.norm(p_rel)/max(np.linalg.norm(v_rel),1)
        if t_go < t_cut: return np.zeros(2)
        omega = np.cross(p_rel, v_rel)/np.linalg.norm(p_rel)**2
        a_cmd = 3*np.cross(v_rel, omega)
        e_y = np.cross([0,0,1], v_now); e_y/=np.linalg.norm(e_y)
        e_z = np.cross(v_now, e_y); e_z/=np.linalg.norm(e_z)
        u = np.array([np.dot(e_z,a_cmd)-g_bar*cg, np.dot(e_y,a_cmd)])
        nrm = np.linalg.norm(u); return u*u_max/nrm if nrm>u_max else u

    def simulate(self, p_t, v_t=np.zeros(3), init_state=None, v_w0=None):
        if init_state is None:
            init_state=[1000.0, 0.6981317007977318, -1.952812610696693, 0.0, 0.0, 0.0]
        if v_w0 is None:
            v_w0=5*np.random.randn(3); v_w0[-1]=0
        t_arr = np.arange(0, self.tf, self.dt)
        state = np.zeros((len(t_arr),6)); state[0]=init_state
        v_w = v_w0.copy()
        deriv_prev = self._state_derivative(state[0], v_w,
                                            self.compute_guidance_cmd(0,state[0],p_t,v_t))
        for k in range(len(t_arr)-1):
            u = self.compute_guidance_cmd(t_arr[k],state[k],p_t,v_t)
            deriv = self._state_derivative(state[k], v_w, u)
            state[k+1] = state[k] + self.dt*(3*deriv-deriv_prev)/2
            v_w[:2]+=np.random.randn(2); deriv_prev=deriv
            if state[k,-1] < 0:
                a_1=state[k,-1]
                a_2=state[k-1,-1]
                b_1=state[k,:]
                b_2=state[k-1,:]
                prob=a_2/(a_2-a_1)
                state[k,:]=prob*(b_1-b_2) +b_2
                break
        t_arr = t_arr[:k+1]
        state = state[:k+1]
        return t_arr, state

    def monte_carlo(self, targets, n=200):
        out=[]
        for p_t in targets[:n]:
            #psi0=math.atan2(p_t[0], p_t[1])+np.pi/2
            init = [1000.0, 0.6981317007977318, -1.952812610696693, 0.0, 0.0, 0.0]
            _, st=self.simulate(p_t, init_state=init)
            out.append(st[:,3:6])
        return out

    # ─────────────────── 3) 데이터 손상 ───────────────────────────
    @staticmethod
    def corrupt(trj, noise_std=2000, missing_ratio=.2):
        noisy=trj+np.random.normal(scale=noise_std,size=trj.shape)
        idx=np.random.choice(trj.shape[0],int(missing_ratio*trj.shape[0]),False)
        noisy[idx]=0.0; return noisy

    # ─────────────────── 4) ADMM 재구성 ───────────────────────────
    @staticmethod
    def _second_diff(n):
        D=np.zeros((n-2,n));
        for i in range(n-2): D[i,i:i+3]=(1,-2,1)
        return D

    def admm_reconstruct(self, y, lam1=4500, lam2=1e-4, lamD=30000,
                         rho=.1, max_iter=30, x0=None, r0=None, u0=None, L_pinv = None, L_T_pinv = None):
        n=len(y); D_s=block_diag(*[self._second_diff(n//3)]*3)
        L=np.linalg.cholesky(2*lamD*(D_s.T@D_s)+rho*np.eye(n))
        x=np.zeros(n) if x0 is None else x0.copy()
        r=np.zeros(n) if r0 is None else r0.copy()
        u=np.zeros(n) if u0 is None else u0.copy()
        alpha,beta,inv=lam2/rho,lam1/rho,1/(1+lam2/rho)
        L_psd_inv = np.linalg.pinv(L) if L_pinv is None else L_pinv.copy()
        L_psd_T_inv = np.linalg.pinv(L.T) if L_T_pinv is None else  L_T_pinv.copy()
        for _ in range(max_iter):
            rhs=rho*(y+r-u); y_=L_psd_inv@rhs; x=L_psd_T_inv@y_
            v=x-y+u; s=np.sign(v)*np.maximum(np.abs(v)-beta,0)
            r=inv*s; u+=x-y-r
        return x,r,u , L_psd_inv ,L_psd_T_inv

    def sliding_reconstruct(self, y_obs, y_true, win=100, step=10, **admm_kw):
        T=len(y_obs)//3; n_win=(T-win)//step+1
        metr=np.zeros((n_win,3,3)); x_opt=np.zeros_like(y_obs)
        x_prev=r_prev=u_prev=None
        LP = None
        LTP = None
        x_opt_save = []
        for w,st in enumerate(range(0,T-win+1,step)):
            pct=100*(w+1)/n_win; print(f"\r[ADMM] {pct:6.2f}% ({w+1}/{n_win})",end="",flush=True)
            segs=[y_obs[k*T+st:k*T+st+win] for k in range(3)]; y_w=np.hstack(segs)
            x_prev,r_prev,u_prev,LP, LTP =self.admm_reconstruct(y_w,x0=x_prev,r0=r_prev,u0=u_prev,**admm_kw,L_pinv =LP, L_T_pinv = LTP)
            x_opt_save.append(x_prev)
            for k in range(3):
                x_opt[k*T+st:k*T+st+win]=x_prev[k*win:(k+1)*win]
                
            tru=np.hstack([y_true[k*T+st:k*T+st+win] for k in range(3)])
            for ax in range(3):
                err=x_prev[ax*win:(ax+1)*win]-tru[ax*win:(ax+1)*win]
                metr[w,ax,0]=np.sqrt(np.mean(err**2))
                metr[w,ax,1]=np.mean(np.abs(err))
                metr[w,ax,2]=np.max(np.abs(err))
        print()
        return x_opt, metr , x_opt_save

    # ─────────────────── 5) RMSE/MAE/MAX 플롯 ─────────────────────
    @staticmethod
    def plot_metric(t, metr, names=("RMSE","MAE","MAX")):
        lab, col = ['X','Y','Z'], ['b','orange','g']
        for i,name in enumerate(names):
            plt.figure(figsize=(8,3))
            for ax in range(3):
                plt.plot(t, metr[:,ax,i], col[ax], label=lab[ax])
            plt.title(name); plt.xlabel('time (s)'); plt.grid(alpha=.3); plt.legend(); plt.tight_layout(); plt.show()

# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    pipe=TrajectoryReconPipeline()

    launches=pipe.make_launch_points(1)
    targets,_=pipe.make_targets(1)
    targets = np.array([[-20713.37118189 ,-51557.53484353   ,   0.        ]])
    trj=pipe.monte_carlo(targets,1)[0]
    launch_p = np.array([155942.39816931,  61204.72051727,      0.        ])
    trj += launch_p
    print(len(trj))
    corrupted=pipe.corrupt(trj, noise_std=2000, missing_ratio=.2)
    y_obs=np.hstack(corrupted.T); y_true=np.hstack(trj.T)

    # ADMM 복원
    x_hat, metr,x_opt_save=pipe.sliding_reconstruct(
        y_obs,y_true,win=100,step=10,
        lam1=4500,lam2=1e-4,lamD=30000,rho=.1,max_iter=10)

    # ───── 결과 시각화 ───────────────────────────────────────
    T     = len(y_obs) // 3
    Time  = np.arange(T) * pipe.dt
    win   = 100            # 사용한 window 크기
    step  = 10             # 사용한 step  간격

    fig, axes = plt.subplots(3, 1, figsize=(12, 6), sharex=True)
    labels = ['X', 'Y', 'Z']
    colors = ['tab:blue', 'tab:orange', 'tab:green']

    for k in range(3):
        # 1) 원 데이터 / 복원 / 진실
        axes[k].plot(Time, y_obs[k*T:(k+1)*T],  color=colors[k], alpha=0.5, label='obs')
        axes[k].plot(Time, y_true[k*T:(k+1)*T], '--', color='k',       lw=0.8,   label='true')


    for i in range(0,len(x_opt_save)):
        x_opt = x_opt_save[i]
        for j in range(3):
            time_arr = Time[i*step:i*step+win]
            axes[j].plot(time_arr,
            x_opt[len(x_opt)//3 * j :len(x_opt)//3 * (j+1) ],
            color=colors[j], linestyle='--', alpha=0.9, linewidth=1)

                    
        axes[k].set_ylabel(labels[k])
        axes[k].legend(loc='upper right')
        axes[k].grid(alpha=.3)

    axes[-1].set_xlabel('time (s)')
    plt.legend()
    plt.tight_layout()
    plt.show()
