import numpy as np, sys, itertools
from multiprocessing import Pool
from botsim import BotSim, LIB
def stand(a):
    seed,side,ra,hold=a; s=BotSim(seed=seed,randomize=True); LIB.bot_recover(s.ctl,ra,hold); s.place(1.8*side); s.run(2.5)
    L=np.array(s.log); st=L[:,1].astype(int)
    first=(np.diff((st==2).astype(int))==1).sum()==1
    return bool(L[-1,1]==4 and first)
if __name__=="__main__":
    N=20
    with Pool(2) as p:
        for ra,hold in itertools.product([0.5,1.0,3.0,6.0],[0.0,0.3]):
            u=p.map(stand,[(7000+i,sd,ra,hold) for i in range(N) for sd in (1,-1)])
            print(ra,hold,f"first-try stand-up: {sum(u)}/{2*N}",flush=True)
