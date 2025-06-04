import numpy as np
import torch
from torch import nn
from torch.utils.data import TensorDataset, DataLoader

from tr import TrajectoryReconPipeline


def generate_dataset(n_samples=200, window=50):
    pipe = TrajectoryReconPipeline()
    targets, _ = pipe.make_targets(n_samples)
    trajectories = pipe.monte_carlo(targets, n_samples)
    X, y = [], []
    for trj in trajectories:
        if len(trj) >= window:
            X.append(trj[-window:].reshape(-1))
            y.append(trj[-1][:2])
    return np.stack(X), np.stack(y)


class SimpleImpactNet(nn.Module):
    def __init__(self, input_dim, hidden=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 2),
        )

    def forward(self, x):
        return self.net(x)


def main():
    window = 50
    X, y = generate_dataset(200, window)
    dataset = TensorDataset(torch.from_numpy(X).float(),
                            torch.from_numpy(y).float())
    loader = DataLoader(dataset, batch_size=32, shuffle=True)

    model = SimpleImpactNet(X.shape[1])
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    for epoch in range(10):
        for xb, yb in loader:
            optimizer.zero_grad()
            pred = model(xb)
            loss = loss_fn(pred, yb)
            loss.backward()
            optimizer.step()
        print(f"epoch {epoch+1}, loss {loss.item():.4f}")

    torch.save(model.state_dict(), "impact_model.pth")
    print("saved model to impact_model.pth")


if __name__ == "__main__":
    main()
