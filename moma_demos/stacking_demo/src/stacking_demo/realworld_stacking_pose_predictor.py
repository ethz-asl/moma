import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import os
import numpy as np
import copy
import json
import matplotlib.pyplot as plt
import argparse

# Define the MLP model
class MLPModel(nn.Module):
    def __init__(self, input_size, hidden_sizes, output_size=1):
        super(MLPModel, self).__init__()
        layers = []
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(input_size, hidden_size))
            layers.append(nn.ReLU())
            input_size = hidden_size
        layers.append(nn.Linear(input_size, output_size))
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)

# Dataset class for obj_dict, for now only makes sense for I-shaped objects
class ObjDictDataset(Dataset):
    def __init__(self, obj_dicts, cube_len, encoding_type, latent_size_obj):
        self.data = obj_dicts
        self.cube_len = cube_len
        self.encoding_type = encoding_type
        self.latent_size_obj = latent_size_obj

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        obj_dict = self.data[idx]
        latent_encoding = torch.tensor(obj_dict["latent_encoding"], dtype=torch.float32)
        if self.encoding_type == "random": # Generate a different random encoding every time the object is loaded
            latent_encoding = torch.rand(self.latent_size_obj, dtype=torch.float32).tolist()
        # ground_truth_com = self.calculate_com(obj_dict["cube_weights"])
        return latent_encoding, obj_dict["cube_weights"]

    @staticmethod
    def calculate_com(cube_weights, cube_len): 
        nr_cubes = len(cube_weights) 
        center_positions = torch.arange(1/2, nr_cubes + 1/2, dtype=torch.float32) * cube_len
        com_from_left = torch.dot(torch.tensor(cube_weights), center_positions) / sum(cube_weights)
        com = com_from_left - cube_len * nr_cubes / 2
        return com.item(), com_from_left

class TowerDictDataset(Dataset):
    def __init__(self, tower_dicts, max_tower_height, latent_size_obj, cube_len, encoding_type, encoding_info):
        self.data = tower_dicts
        self.max_tower_height = max_tower_height
        self.latent_size_obj = latent_size_obj
        self.cube_len = cube_len
        self.encoding_type = encoding_type
        self.encoding_info = encoding_info

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        tower_dict = self.data[idx]
        latent_encoding = self.get_latent_tower(tower_dict, self.encoding_type, self.encoding_info)
        tower_cube_weights = self.get_tower_cube_weights(tower_dict)
        return latent_encoding, tower_cube_weights, tower_dict["obj_pos"]

    def get_latent_tower(self, tower_dict, encoding_type, encoding_info): 
        nr_objects = len(tower_dict) - 1 #the last one is obj_pos
        tower_latent_encoding = []
        # PW TODO for now we just append the object latent encodings and the positions. 
        # In the future we could have a better preprocessing step for the tower latent to reduce dimensionality already
        # Also, maybe we could have a separate shape latent encoding of the pointcloud of the whole tower, plus the physics encoding of the individual objects plus position
        for i in range(nr_objects):
            if encoding_type == "oracle":
                obj_latent = get_oracle_latent_obj([tower_dict["obj_"+str(i)]], self.cube_len)[0]["latent_encoding"] #this return (com, obj_len)
            elif encoding_type == "latent":
                obj_latent = tower_dict["obj_"+str(i)]["latent_encoding"]
            elif encoding_type == "random":
                obj_latent = torch.rand(self.latent_size_obj, dtype=torch.float32)

            nr_of_cubes = len(tower_dict["obj_"+str(i)]["cube_weights"])
            obj_len = nr_of_cubes * self.cube_len 
            full_obj_in_tower_info = self.get_full_obj_in_tower_info(i, obj_len, obj_latent, encoding_info, tower_dict["obj_pos"][i])
            tower_latent_encoding.extend(full_obj_in_tower_info)  # Append the full object info to the tower encoding

        return torch.tensor(tower_latent_encoding, dtype=torch.float32)

    @staticmethod
    def get_full_obj_in_tower_info(i, obj_len, obj_latent, encoding_info, obj_pos=None):

        # Encoding for "all": height, latent, length, position
        full_obj_info = [i]
        if encoding_info == "all" or encoding_info == "physics_only":
            full_obj_info.extend(obj_latent)
        if encoding_info == "all" or encoding_info == "vision_only":
            full_obj_info.append(obj_len)
        
        if obj_pos is not None: # if we come from get_latent_tower this will not be none, but for combine_latent_encodings it will be
            full_obj_info.append(obj_pos)
        return full_obj_info

    @staticmethod
    def get_tower_cube_weights(tower_dict):
        tower_cube_weights = []
        nr_objects = len(tower_dict) - 1 #the last one is obj_pos
        for i in range(nr_objects):
            tower_cube_weights.append(tower_dict["obj_"+str(i)]["cube_weights"])
        return tower_cube_weights

    @staticmethod
    def calculate_com_tower(tower_cube_weights, tower_obj_pos, cube_len):
        height_of_stack = len(tower_obj_pos)
        com_tower = 0
        cube_weights_tot = 0

        for i in range(height_of_stack):
            cube_weights_i = tower_cube_weights[i]
            obj_i_com, _ = ObjDictDataset.calculate_com(cube_weights_i, cube_len)
            obj_i_pos = tower_obj_pos[i]
            com_tower += (obj_i_pos + obj_i_com) * np.sum(cube_weights_i)
            cube_weights_tot += np.sum(cube_weights_i)
        
        com_tower = com_tower/cube_weights_tot
        return com_tower
        

# Define the training pipeline
class StackingPosePredictor:
    def __init__(self, latent_size_obj, latent_size_tower, latent_size_total, hidden_sizes, cube_len, learning_rate=0.001):
        self.latent_size_tower = latent_size_tower
        self.latent_size_obj = latent_size_obj
        self.latent_size_total = latent_size_total
        self.model = MLPModel(self.latent_size_total, hidden_sizes) 
        self.cube_len = cube_len


    def predict(self, obj_latent_encoding, tower_latent_encoding, obj_cube_weights, tower_current_height, encoding_info):
        latent_encoding = self.combine_latent_encodings(obj_latent_encoding, tower_latent_encoding, obj_cube_weights, tower_current_height, encoding_info)
        self.model.eval()
        with torch.no_grad():
            return self.model(latent_encoding)

    def combine_latent_encodings(self, obj_latent_encoding, tower_latent_encoding, obj_cube_weights, tower_current_height, encoding_info):
        new_obj_height = tower_current_height
        obj_len = len(obj_cube_weights) * self.cube_len
        new_obj_full_info = TowerDictDataset.get_full_obj_in_tower_info(new_obj_height, obj_len, obj_latent_encoding, encoding_info)
        latent_encoding = torch.cat((tower_latent_encoding, torch.tensor(new_obj_full_info, dtype=torch.float32))) #TODO in the future we could have a better preprocessing step for the tower latent to reduce dimensionality already      
        # zero padding
        latent_encoding = torch.nn.functional.pad(latent_encoding, (0,self.latent_size_total - len(latent_encoding)), "constant", 0)  # zero padding
        return latent_encoding

def check_stability(stacking_pose_pred, obj_cube_weights, tower_cube_weights, tower_obj_pos, cube_len):
    # This function checks if the predicted stacking pose is stable
    new_obj_fell_down = False
    size_top_obj = cube_len * len(tower_cube_weights[-1])
    pos_top_obj = tower_obj_pos[-1]
    com_new_obj, _ = ObjDictDataset.calculate_com(obj_cube_weights, cube_len)
    com_placed_obj = stacking_pose_pred + com_new_obj # in tower frame
    if com_placed_obj < (pos_top_obj - size_top_obj/2) or com_placed_obj > (pos_top_obj + size_top_obj/2):
        new_obj_fell_down = True
        # print("New object fell down")

    # Check if the tower collapsed
    # Add object to the tower
    new_tower_cube_weights = copy.deepcopy(tower_cube_weights)
    new_tower_obj_pos = copy.deepcopy(tower_obj_pos)
    new_tower_cube_weights.append(obj_cube_weights)  # add the new object to the tower
    new_tower_obj_pos.append(stacking_pose_pred)  # add the predicted stacking pose to the tower
    
    # For this we check for each height if the center of mass of the above tower is within the bounds of its bottom object
    tower_collapsed = False
    for i in range(1, len(new_tower_obj_pos)):
        size_bottom_obj = cube_len * len(new_tower_cube_weights[i-1])
        pos_bottom_obj = new_tower_obj_pos[i-1]
        com_sub_tower = TowerDictDataset.calculate_com_tower(new_tower_cube_weights[i:], new_tower_obj_pos[i:], cube_len)
        tower_collapsed = com_sub_tower < (pos_bottom_obj - size_bottom_obj/2) or com_sub_tower > (pos_bottom_obj + size_bottom_obj/2)
        if tower_collapsed:
            break

    return new_obj_fell_down, tower_collapsed


def load_obj_dicts_from_json(json_path, realworld_exp=False): 
    # Load the object dictionaries from a JSON file
    with open(json_path, 'r') as f:
        obj_dicts = json.load(f)
    cube_list = []
    for obj_dict in obj_dicts:
        cube_list.extend(obj_dict["cube_weights"])
    cube_list = list(set(cube_list))  # Remove duplicates
    cube_list.sort()  # Sort the cube weights
    if realworld_exp:
        desired_weights = [[cube_list[0], cube_list[0]], [cube_list[0], cube_list[2]], [cube_list[0], cube_list[3]]]
        # Filter the object dictionaries to only include the desired weights
        obj_dicts = [obj_dict for obj_dict in obj_dicts if obj_dict["cube_weights"] in desired_weights]

    return obj_dicts, cube_list

def get_oracle_latent_obj(obj_dicts, cube_len):
    for i in range(len(obj_dicts)):
        # oracle encoding: [com_from_geometric_center, nr_of_cubes]
        com, _ = ObjDictDataset.calculate_com(obj_dicts[i]["cube_weights"], cube_len)
        obj_dicts[i]["latent_encoding"] = [com]  
    return obj_dicts

def get_stable_tower_dicts(data_path):
    stable_tower_dicts = json.load(open(data_path, 'r'))
    return stable_tower_dicts


def load_datasets(tower_data_path, obj_data_path, latent_size_obj, cube_len, encoding_type, realworld_exp):

    obj_dicts, cube_list = load_obj_dicts_from_json(obj_data_path, realworld_exp)  # Load the object dictionaries from a JSON file
    
    if encoding_type == "oracle":
        # Get the oracle latent encoding for the objects, ie com
        obj_dicts = get_oracle_latent_obj(obj_dicts, cube_len)
    elif encoding_type == "random":
        # Randomly generate latent encodings for the objects
        for obj_dict in obj_dicts:
            obj_dict["latent_encoding"] = torch.rand(latent_size_obj, dtype=torch.float32).tolist()
    elif encoding_type == "latent":
        # Ensure that the objects have a latent encoding already
        for obj_dict in obj_dicts:
            if "latent_encoding" not in obj_dict:
                raise ValueError("Objects must have a latent encoding for 'latent' encoding type.")
    # Load towers from dataset.
    stable_tower_dicts = get_stable_tower_dicts(tower_data_path)

    return obj_dicts, stable_tower_dicts, cube_list


def visualize_prediction(stacking_pose_pred, obj_cube_weights, tower_cube_weights, tower_obj_pos, cube_len, cube_list, new_obj_fell_down, tower_collapsed, poster_visualization=False):
    # print(f"Predicted stacking pose: {stacking_pose_pred.item()}")
    # PW for now hardcoded, in the future scale with the min and max values of cube_weights, or add the materials plus weights in yaml
    # The darker the color, the heavier the cube
    weight_alphas = {}
    for i in range(len(cube_list)):
        weight_alphas[str(cube_list[i])] = 0.4 + i * 0.1
    
    fig, ax = plt.subplots(figsize=(6, 8))

    # Plot the existing tower
    current_height = 0
    for height_idx, cube_weights in enumerate(tower_cube_weights):
        for cube_idx in range(len(cube_weights)):
            rect = plt.Rectangle((tower_obj_pos[height_idx] + cube_len*(-len(cube_weights)/2 + cube_idx), current_height), cube_len, cube_len, edgecolor='black', facecolor="#1008F0", alpha=weight_alphas[str(cube_weights[cube_idx])])
            ax.add_patch(rect)
        # plot the center of mass of each object
        com, com_from_left = ObjDictDataset.calculate_com(cube_weights, cube_len)
        plt.plot(tower_obj_pos[height_idx]+com, current_height + cube_len / 2, marker='x', color='black', markersize=3, linewidth=2)
        current_height += cube_len

    # Plot the predicted cube
    predicted_pos = stacking_pose_pred
    if tower_collapsed or new_obj_fell_down:
        # If the object is predicted to fall down, we plot it in a different color
        predicted_color = "#DC1212"
    else:
        predicted_color = "#41A631"
    for cube_idx in range(len(obj_cube_weights)):
        rect = plt.Rectangle((predicted_pos + cube_len*(-len(obj_cube_weights)/2 + cube_idx), current_height), cube_len, cube_len, edgecolor='black', facecolor=predicted_color, alpha=weight_alphas[str(obj_cube_weights[cube_idx])])
        ax.add_patch(rect)
    
    # plot the center of mass of the object
    com, com_from_left = ObjDictDataset.calculate_com(obj_cube_weights, cube_len)
    ax.plot(predicted_pos + com, current_height + cube_len / 2,  marker='x', color='black', markersize=3, linewidth=2)
    
    # Set plot limits and labels
    ax.set_xlim(-5*cube_len, 5*cube_len)
    ax.set_ylim(0, current_height + cube_len)
    ax.set_aspect('equal', adjustable='box')
    # ax.set_xlabel("Position (m)")
    # ax.set_ylabel("Height (m)")
    ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
    ax.tick_params(axis='y', which='both', left=False, right=False, labelleft=False, labelright=False)
    if not poster_visualization:
        if new_obj_fell_down: 
            ax.set_title("Fail: Object will fall down")
        elif tower_collapsed:
            ax.set_title("Fail: Tower will collapse")
        else:
            ax.set_title("Stable Stacking Pose")
    # plt.grid(True)
    # Add text below the plot
    # plt.figtext(0.1, 0.25, 'Predicted Object (Red) on existing tower (Blue). \nThe darker the color, the heavier the cube.', 
    #             ha='left', fontsize=12, color='black')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    plt.show()

def run_realworld_stacking_pose_predictor(obj_id, checkpoint_path="/root/moma_ws/src/moma/moma_demos/stacking_demo/src/stacking_demo/stacking_pose_predictor_data/checkpoints", data_folder="/root/moma_ws/src/moma/moma_demos/stacking_demo/src/stacking_demo/stacking_pose_predictor_data/encodings", visualize=False):
    # Fixed Hyperparameters
    cube_len = 0.04 #[m]
    max_tower_height = 3
    realworld_exp = True # has to be true
    hidden_sizes =[12] # [36,12,4] hidden layer sizes
    encoding_type = "latent" # can be changed between "latent", "oracle" or "random"
    encoding_info = "all" # can not be changed in this script

    if encoding_type == "oracle": # if oracle encoding, we only use the com and nr of cubes as the latent encoding
        latent_size_obj = 1 #this is the CoM
    elif encoding_type == "latent" or encoding_type == "random":
        latent_size_obj = 8

    # Encoding for "all": height, latent, length, position
    latent_size_tower = max_tower_height * (latent_size_obj + 3) # height, latent, length, position
    latent_size_total = latent_size_tower + latent_size_obj + 2 # latent+2 (height, length)

    experiment_name = "realworld_exp_latent_all"

    # encoding_test_data_path = "/home/paula/Code/latobj/results/paula_selection/encoding_saver/test_encodings.json"
    encoding_test_data_path = os.path.join(data_folder, "latobj/test_encodings.json")
    tower_test_data_path = os.path.join(data_folder,"realworld_experiment_tower.json")

    # Initialize the pipeline
    predictor = StackingPosePredictor(latent_size_obj, latent_size_tower, latent_size_total, hidden_sizes, cube_len)

    # Load the model for inference
    predictor.model.load_state_dict(torch.load(os.path.join(checkpoint_path, experiment_name) + "_best.pth"))
    print("Loading model from", os.path.join(checkpoint_path, experiment_name) + "_best.pth")
    # Prediction example
    # TODO currently the objects are the same in train and test, I should include unseen objects in the test set
    test_obj_dicts, test_stable_tower_dicts, cube_list = load_datasets(tower_test_data_path, encoding_test_data_path, latent_size_obj, cube_len, encoding_type, realworld_exp)
    obj_id_to_weights = {0:  [cube_list[0], cube_list[0]], 1:  [cube_list[0], cube_list[2]], 2:  [cube_list[0], cube_list[3]]}  # Mapping of object IDs to weights
    
    test_obj_dataset = ObjDictDataset(test_obj_dicts, cube_len, encoding_type, latent_size_obj)
    test_tower_dataset = TowerDictDataset(test_stable_tower_dicts, max_tower_height, latent_size_obj, cube_len, encoding_type, encoding_info)
    print("Loaded test dataset from", tower_test_data_path)
    print(f"Number of test objects: {len(test_obj_dataset)}")
    print(f"Number of test towers: {len(test_tower_dataset)}")
    print("ready to start predictions...")

    # get a random object idx that fulfills the requirements to be an obj_id
    # not very efficient, but works for now
    obj_cube_weights = [0]
    while obj_cube_weights != obj_id_to_weights[obj_id]:
        obj_idx = np.random.randint(0, len(test_obj_dataset))
        obj_latent_encoding, obj_cube_weights = test_obj_dataset[obj_idx]

    tower_latent_encoding, tower_cube_weights, tower_obj_pos = test_tower_dataset[0]
    tower_current_height = len(tower_obj_pos)  # current height of the tower
    # get the stacking pose prediction
    stacking_pose_pred = predictor.predict(obj_latent_encoding, tower_latent_encoding, obj_cube_weights, tower_current_height, encoding_info)
    
    if visualize:
        new_obj_fell_down, tower_collapsed = check_stability(stacking_pose_pred.item(), obj_cube_weights, tower_cube_weights, tower_obj_pos, cube_len)
        visualize_prediction(stacking_pose_pred.item(), obj_cube_weights, tower_cube_weights, tower_obj_pos, cube_len, cube_list, new_obj_fell_down, tower_collapsed)

    print(f"Predicted stacking position for object {obj_idx}: {stacking_pose_pred.item()}")

    return stacking_pose_pred.item()  # Return the predicted stacking position

# Example usage
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stacking Pose Predictor")
    parser.add_argument("--visualize", action="store_true", help="Visualize predictions")
    parser.add_argument("--checkpoint_path", type=str, default="/root/moma_ws/src/moma/moma_demos/stacking_demo/src/stacking_demo/stacking_pose_predictor_data/checkpoints", help="Path to save model checkpoints")
    parser.add_argument("--data_folder", type=str, default="/root/moma_ws/src/moma/moma_demos/stacking_demo/src/stacking_demo/stacking_pose_predictor_data/encodings", help="Folder containing training data")
    parser.add_argument("--obj_id", type=int, default=0, help="Object ID to use for prediction, 0,1 or 2")
    args = parser.parse_args()

    visualize = args.visualize
    checkpoint_path = args.checkpoint_path
    data_folder = args.data_folder
    obj_id = args.obj_id
    # Run the real world stacking pose predictor
    stacking_position = run_realworld_stacking_pose_predictor(obj_id, checkpoint_path, data_folder, visualize)
