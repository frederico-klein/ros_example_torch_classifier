#!/usr/bin/env python3
# -*- coding: utf-8 -*-

### here we read text and publish something
### need to add support for transformers tokenizer as well

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ros_example_torch_classifier.msg import StringStamped

from keras.preprocessing.text import Tokenizer
from keras.preprocessing import sequence


###if this needs everything, and I think it does, then, it makes almost no sense for it to be a standalone thing

# create a tokenizer
token = Tokenizer()

token.fit_on_texts(df[TEXT]) ##TEXT stands for titleabstract
word_index = token.word_index

# convert text to sequence of tokens and pad them to ensure equal length vectors
#train_seq_x = sequence.pad_sequences(token.texts_to_sequences(train_x), maxlen=300)
#valid_seq_x = sequence.pad_sequences(token.texts_to_sequences(valid_x), maxlen=300)


rospy.init_node("tokenizer")
        
rospy.Subscriber("text", StringStamped, tokenize)
mypub = rospy.Publisher("array", Float64MultiArray)


def tokenize(text):
    tokenized = sequence.pad_sequences(token.texts_to_sequences(text), maxlen=300)
 
    thisArray = Float64MultiArray()
    thisArray.data = tokenized
    ##maybe copy the header
    mypub.publish(thisArray)

