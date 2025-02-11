from segment_anything import SamAutomaticMaskGenerator, sam_model_registry
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

# Define green color range (adjust if needed)
lower_green = np.array([35, 40, 40])    # Lower bound of green in HSV
upper_green = np.array([85, 255, 255])  # Upper bound of green in HSV
# green_mask = cv2.inRange(hsv, lower_green, upper_green)

sam = sam_model_registry["vit_h"](checkpoint="/root/moma_ws/src/moma/moma_ui/sam_models/sam_vit_h_4b8939.pth")
# sam = sam_model_registry["vit_l"](checkpoint="/root/moma_ws/src/ros_sam/ros_sam/models/sam_vit_l_0b3195.pth")

device = "cuda"
sam = sam.to(device)

mask_generator = SamAutomaticMaskGenerator(
    model=sam,
    points_per_side=32,
    pred_iou_thresh=0.86,
    stability_score_thresh=0.92,
    crop_n_layers=1,
    crop_n_points_downscale_factor=2,
    min_mask_region_area=100,  # Requires open-cv to run post-processing
)

# mask_generator = SamAutomaticMaskGenerator(sam)
img_path = '/root/moma_ws/bags/real_images/images/'

def show_anns(anns):
    if len(anns) == 0:
        return
    sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((sorted_anns[0]['segmentation'].shape[0], sorted_anns[0]['segmentation'].shape[1], 4))
    img[:,:,3] = 0
    for ann in sorted_anns:
        m = ann['segmentation']
        color_mask = np.concatenate([np.random.random(3), [0.35]])
        img[m] = color_mask
    return img

image_dir = '/root/moma_ws/bags/real_images/2025-02-10_18-12-38/elevmap_raw_img/'
output_dir = '/root/moma_ws/bags/real_images/sam/2025-02-10_18-12-38/elevmap_raw_img/output/'

# create output directory if it does not exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# create NxN grid of the raw images
n_col = 4
n_row = len(os.listdir(image_dir)) // n_col + 1
fig, axs = plt.subplots(n_row, n_col, figsize=(20, 20))

fig2, axs2 = plt.subplots(n_row, n_col, figsize=(20, 20))

fig3, axs3 = plt.subplots(n_row, n_col, figsize=(20, 20))

fig4, axs4 = plt.subplots(n_row, n_col, figsize=(20, 20))

fig5, axs5 = plt.subplots(n_row, n_col, figsize=(20, 20))

fig6, axs6 = plt.subplots(n_row, n_col, figsize=(20, 20))

img_cnt = 0
for filename in os.listdir(image_dir):
    if filename.endswith('.jpg') or filename.endswith('.png'):
        image_path = os.path.join(image_dir, filename)
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # scale up image by 20
        # image = cv2.resize(image, (image.shape[1]*20, image.shape[0]*20))

        masks = mask_generator.generate(image)

        axs[img_cnt // n_col, img_cnt % n_col].imshow(image)
        axs[img_cnt // n_col, img_cnt % n_col].axis('off')
        
        # generate masks
        masks = mask_generator.generate(image)
        mask_img = show_anns(masks)
        axs2[img_cnt // n_col, img_cnt % n_col].imshow(mask_img)
        axs2[img_cnt // n_col, img_cnt % n_col].axis('off')     

        # overlay masks on the raw image

        # print number of masks
        print(f'Number of masks: {len(masks)}')
        # Ensure the mask has the same shape as the image
        mask_rgb = mask_img[:, :, :3]*255.0  # Take only the RGB channels
        mask_alpha = mask_img[:, :, 3:]  # Extract the alpha channel (transparency)

        # Blend using alpha from the mask
        blended = cv2.addWeighted(image, 1, mask_rgb, 0.5,
                                    0, dtype=cv2.CV_8U)
    

        axs3[img_cnt // n_col, img_cnt % n_col].imshow(blended)
        axs3[img_cnt // n_col, img_cnt % n_col].axis('off')

        # apply green mask
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        background_green = cv2.bitwise_and(image, image, mask=green_mask)
        foreground = cv2.bitwise_and(image, image, mask=cv2.bitwise_not(green_mask))

        axs4[img_cnt // n_col, img_cnt % n_col].imshow(background_green)
        axs4[img_cnt // n_col, img_cnt % n_col].axis('off')

        axs5[img_cnt // n_col, img_cnt % n_col].imshow(foreground)
        axs5[img_cnt // n_col, img_cnt % n_col].axis('off')

        # overlay
        img_masked_by_green = cv2.addWeighted(image, 1, background_green, 0.5, 0, dtype=cv2.CV_8U)
        axs6[img_cnt // n_col, img_cnt % n_col].imshow(img_masked_by_green)
        axs6[img_cnt // n_col, img_cnt % n_col].axis('off')
               
                
        img_cnt += 1

# remove empty subplots
for i in range(img_cnt, n_col * n_row):
    fig.delaxes(axs[i // n_col, i % n_col])
    fig2.delaxes(axs2[i // n_col, i % n_col])
    fig3.delaxes(axs3[i // n_col, i % n_col])
    fig4.delaxes(axs4[i // n_col, i % n_col])
    fig5.delaxes(axs5[i // n_col, i % n_col])
    fig6.delaxes(axs6[i // n_col, i % n_col])
# title
fig.suptitle('Raw Images', fontsize=20)
fig2.suptitle('Segmentation Masks', fontsize=20)
fig3.suptitle('Raw Images with Masks', fontsize=20)
fig4.suptitle('Background (Green)', fontsize=20)
fig5.suptitle('Foreground', fontsize=20)
fig6.suptitle('Background (Green) Overlay', fontsize=20)

# plt.show()

# save the images
fig.savefig(os.path.join(output_dir, 'raw_images.png'))
fig2.savefig(os.path.join(output_dir, 'segmentation_masks.png'))
fig3.savefig(os.path.join(output_dir, 'raw_images_with_masks.png'))
fig4.savefig(os.path.join(output_dir, 'bg_green.png'))
fig5.savefig(os.path.join(output_dir, 'foreground.png'))
fig6.savefig(os.path.join(output_dir, 'bg_green_overlay.png'))