import numpy as np, math, sys
from multiprocessing import Pool
from botsim import BotSim, STATES
def turn(seed):
    s=BotSim(seed=seed,randomize=True); s.place(0); s.hold(); s.run(1.5)
    s.run(3.0, mode=2, angle=math.radians(90), seq=1); s.run(3.0, mode=2, angle=math.radians(-180), seq=2)
    s.run(3.0, mode=0, v=0.8, yaw_rate=1.0); s.run(3.0, v=0.0, yaw_rate=0.0)
    L=np.array(s.log); return bool((L[400:,1]==4).all())
def stand(a):
    seed,side=a; s=BotSim(seed=seed,randomize=True); s.place(1.8*side); s.run(4.0)
    L=np.array(s.log); return bool(L[-1,1]==4 and np.abs(L[-500:,2]).max()<0.2)
if __name__=="__main__":
    N=int(sys.argv[1]) if len(sys.argv)>1 else 12
    with Pool(2) as p:
        t=p.map(turn, range(5000,5000+N)); u=p.map(stand,[(6000+i,sd) for i in range(N) for sd in (1,-1)])
    print(f"drive+turn without falling: {sum(t)}/{N}   stand-up: {sum(u)}/{2*N}")
