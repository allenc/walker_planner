import torch
# Training params
TASK_HORIZON = 40
PLAN_HORIZON = 5

# CEM params
POPSIZE = 200
NUM_ELITES = 20
MAX_ITERS = 5

# Model params
EXPERIMENT_PATH_PREFIX = 'experiments/PENN'
INITIAL_LEARNING_RATE = 0.001
LEARNING_RATE_UPDATE_CYCLES = 15000
HIDDEN_LAYERS = [400, 400, 400]
CUDA_AVAILABLE = True and torch.cuda.is_available()
WEIGHT_DECAY = 0.0001
REPLAY_BUFFER_MAX_SIZE = 50000
BATCH_SIZE = 128
USE_TENSORBOARD = True
TF_PATH = 'tensorboard'
X_DIM = 2
C_DIM = 10
LATENT_DIM = 32

# Dims
STATE_DIM = 8




