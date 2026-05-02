import numpy as np, pathlib, collections
ctr = collections.Counter()
for f in sorted(pathlib.Path("~/aic_data_100/raw").expanduser().glob("*.npz")):
    z = np.load(f, allow_pickle=True)
    for t in z["port_types"]:
        ctr[str(t)] += 1
print(f"Files: {len(list(pathlib.Path('~/aic_data_100/raw').expanduser().glob('*.npz')))}")
print(f"Ports by type: {dict(ctr)}")
