import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("reachable_points.csv")

print(df.head())
print(f"\nTotal reachable points: {len(df)}")

# Create figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
ax.scatter(
    df["x"],
    df["y"],
    df["z"],
    s=2
)

# Labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_title("Robot Reachable Workspace")

plt.show()