#!/usr/bin/env python3
# -*- coding: utf-8 -*-

### here we read text and publish something
### need to add support for transformers tokenizer as well

import rospy

from keras.preprocessing.text import Tokenizer
from keras.preprocessing import sequence

from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ros_example_torch_classifier.msg import StringStamped
from ros_example_torch_classifier.srv import Dump# , DumpResponse
import ros_example_torch_classifier.utils as u

class Dumps():
    def __init__(self):
        self.all_text = ""
        self.srvs = u.read_params_with("~dump")

        self.srv_list = []
        for ansrv in self.srvs:
            self.srv_list.append(rospy.ServiceProxy(ansrv, Dump) )

    def update(self):
        ## I don't need this, but it makes thinking about it easier
        self.all_text = ""
        for ansrv in self.srv_list:
            my_response = ansrv()#DumpResponse()
            rospy.logdebug("my response dump response: {}".format(my_response))
            self.all_text+= my_response.data
        #return self.all_text

def tokenize(text):
    tokenized = sequence.pad_sequences(token.texts_to_sequences(text), maxlen=300)
 
    thisArray = Float64MultiArray()
    thisArray.data = tokenized
    ##maybe copy the header
    mypub.publish(thisArray)

###if this needs everything, and I think it does, then, it makes almost no sense for it to be a standalone thing

# create a tokenizer
#token = Tokenizer()

#token.fit_on_texts(df[TEXT]) ##TEXT stands for titleabstract
#word_index = token.word_index

# convert text to sequence of tokens and pad them to ensure equal length vectors
#train_seq_x = sequence.pad_sequences(token.texts_to_sequences(train_x), maxlen=300)
#valid_seq_x = sequence.pad_sequences(token.texts_to_sequences(valid_x), maxlen=300)


rospy.init_node("tokenizer")

token = Tokenizer()

aDump = Dumps()
aDump.update()

token.fit_on_texts(aDump.all_text) ## does this need cleaning?

rospy.Subscriber("text", StringStamped, tokenize)
mypub = rospy.Publisher("array", Float64MultiArray)

rospy.spin()
