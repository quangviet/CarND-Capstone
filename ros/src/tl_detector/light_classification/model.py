############################### Capstone Project ###############################
__author__ = "Vuong Le"
__email__ = "lelordoftech@gmail.com"
__date__ = "04-Nov-2017"
################################################################################


import os
import os.path
import cv2
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Dropout
from keras.layers.convolutional import Convolution2D
from keras.models import load_model
import h5py
from keras import __version__ as keras_version
from keras.layers.advanced_activations import ELU
from keras.callbacks import TensorBoard

import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import collections
import datetime


keras_version = str(keras_version).encode('utf8')
data_0 = './training_data/red/'
data_1 = './training_data/yellow/'
data_2 = './training_data/green/'
data_4 = './training_data/unknown/'
data_paths = [data_0, data_1, data_2, data_4]

model_file_path = 'model.h5'
IMG_COLS = 160
IMG_ROWS = 120
IMG_CH = 3

def getAllSamples(data_paths):
    print('Get All Samples')
    samples = []
    for i in range(4):
        print('    From data path ' + data_paths[i])
        label = [0, 0, 0, 0]
        label[i] = 1
        state = i
        if state == 3:
            state = 4
        paths = os.listdir(data_paths[i])
        for path in paths:
            samples.append([data_paths[i] + path, label, state])
    print('Total samples: ' + str(len(samples)))
    return samples

def visualation_data(samples):
    print('Start Visualation data')
    start = datetime.datetime.now()

    sample_values = [x[2] for x in samples] # get all states

    visual = collections.Counter(sample_values)
    ord_dict = collections.OrderedDict(sorted(visual.items()))
    labels = ord_dict.keys()
    values = ord_dict.values()

    indexes_bar = np.arange(len(labels))
    step_labels = 1
    step_indexes = len(labels)*step_labels/(max(labels) - min(labels))
    new_labels = np.arange(min(labels), max(labels), step_labels)
    indexes = np.arange(0, len(labels), step_indexes)
    width = 1

    fig = plt.figure()
    plt.bar(indexes_bar, values, width)
    plt.xticks(indexes + width * 0.5, labels, rotation=45)
    plt.ylabel('Occurrence')
    plt.xlabel('Traffic light state')
    plt.title('Training examples Visualization')
    plt.minorticks_off()
    plt.tight_layout()
    plt.savefig('./report/training_visualization.png')

    end = datetime.datetime.now()
    diff = end - start
    elapsed_ms = (diff.days * 86400000) + (diff.seconds * 1000) + (diff.microseconds / 1000)
    print('Finish Visualation data in %d ms' %elapsed_ms)

def pre_processing(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    scale_img = cv2.resize(img, (IMG_COLS, IMG_ROWS))
    return scale_img

def getImage(batch_data_path):
    img = cv2.imread(batch_data_path)
    img = pre_processing(img)
    return img

# input  1 x batch_size
# output 3 x batch_size
def generator(samples, batch_size=32):
    num_samples = len(samples)

    while 1:
        shuffle(samples)

        for offset in range(0, num_samples, batch_size):
            images = []
            states = []
            batch_samples = samples[offset:offset+batch_size]

            for batch_sample in batch_samples:
                batch_data_path = batch_sample[0]
                batch_state = batch_sample[1]
                # read in images from cameras
                img = getImage(batch_data_path)
                # add image and angles to data set
                images.append(img)
                states.append(batch_state)

            # Data Augmentation
            augmented_images, augmented_states = [], []
            for image, state in zip(images, states):
                # append original data
                augmented_images.append(image)
                augmented_states.append(state)
                # append augmented data
                augmented_images.append(cv2.flip(image,1))
                augmented_states.append(state)
                augmented_images.append(cv2.flip(image,0))
                augmented_states.append(state)

            X_train = np.array(augmented_images)
            y_train = np.array(augmented_states)

            yield shuffle(X_train, y_train)

def createNewModel():
    print('Create New Model')
    model = None

    if (keras_version == b'2.0.9'):
        model = Sequential()
        # Normalization
        model.add(Lambda(lambda x: x / 255.0 - 0.5,
                        input_shape=(IMG_ROWS,IMG_COLS,IMG_CH)))
        # Resize
        print(model.output_shape)
        # 120x160@3

        # Architecture
        model.add(Convolution2D(filters=24,kernel_size=(5,5),strides=2))
        model.add(ELU())
        model.add(Dropout(0.3))
        print(model.output_shape) # 58x78@24

        model.add(Convolution2D(filters=36,kernel_size=(5,5),strides=2))
        model.add(ELU())
        model.add(Dropout(0.3))
        print(model.output_shape) # 27x37@36

        model.add(Convolution2D(filters=48,kernel_size=(5,5),strides=2))
        model.add(ELU())
        model.add(Dropout(0.3))
        print(model.output_shape) # 12x17@48

        model.add(Convolution2D(filters=64,kernel_size=(5,5)))
        model.add(ELU())
        model.add(Dropout(0.2))
        print(model.output_shape) # 8x13@64

        model.add(Convolution2D(filters=64,kernel_size=(5,5)))
        model.add(ELU())
        model.add(Dropout(0.2))
        print(model.output_shape) # 4x9@64

        model.add(Flatten())
        print(model.output_shape) # 2304

        # default: activation=None
        model.add(Dense(100))
        model.add(ELU())
        model.add(Dropout(0.5))
        print(model.output_shape)

        model.add(Dense(50))
        model.add(ELU())
        model.add(Dropout(0.4))
        print(model.output_shape)

        model.add(Dense(10))
        model.add(ELU())
        model.add(Dropout(0.3))
        print(model.output_shape)

        model.add(Dense(4))
        model.add(ELU())
        print(model.output_shape)

        model.compile(loss='mse', optimizer='adam')
    else:
        print('\033[91m' +
              '[ERROR] Doesn''s support Keras version ' + keras_version +
              '\033[00m')

    # 482,140 parameters.
    return model

def main():
    model = None
    batch_size_val = 3
    num_epochs = 20

    ### Read all driving log from all folder
    samples = getAllSamples(data_paths)

    ### Visualization data
    visualation_data(samples)

    ### Create training samples and validation samples
    train_samples, validation_samples = train_test_split(samples, test_size=0.3)

    ### Create training set and validation set
    train_generator = generator(train_samples, batch_size=int(batch_size_val/3))
    validation_generator = generator(validation_samples, batch_size=int(batch_size_val/3))

    ### Define model
    if os.path.exists(model_file_path) == True: # Load old model
        print('Loading model ' + model_file_path)
        # check that model Keras version is same as local Keras version
        f = h5py.File(model_file_path, mode='r')
        model_version = f.attrs.get('keras_version')
        f.close()

        if model_version != keras_version:
            print('\033[91m' +
                  '[ERROR] You are using Keras version ' + keras_version +
                  ', but the model was built using ' + model_version +
                  '\033[00m')
        else:
            model = load_model(model_file_path)
    else: # Create new model
        print('Creating model ' + model_file_path)
        model = createNewModel()

    if model:
        # Summary representation of the model.
        model.summary()

        ### Train the model using the generator function
        tbCallBack = TensorBoard(log_dir='./Graph', histogram_freq=0,  
              write_graph=True, write_images=True)

        history_object = model.fit_generator(train_generator, \
                    steps_per_epoch=int(len(train_samples)/batch_size_val)+1, \
                    epochs=num_epochs, \
                    verbose=1, \
                    validation_data=validation_generator, \
                    validation_steps=int(len(validation_samples)/batch_size_val)+1, \
                    callbacks=[tbCallBack])

        ### Save model
        print('Saving model ' + model_file_path)
        model.save(model_file_path)

        ### plot the training and validation loss for each epoch
        if history_object:
            fig = plt.figure()
            plt.plot(history_object.history['loss'])
            plt.plot(history_object.history['val_loss'])
            plt.title('model mean squared error loss')
            plt.ylabel('mean squared error loss')
            plt.xlabel('epoch')
            plt.legend(['training set', 'validation set'], loc='upper right')

            ### save data loss
            fig = plt.gcf()
            fig.savefig('./report/model_mean_squared_error_loss.png')
            #plt.show()

    ### Exit
    exit()

if __name__ == '__main__':
    main()
