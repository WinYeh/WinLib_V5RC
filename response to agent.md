response to agent

this document here tracks all the responses to the questions and decisions agents asked me

1. Yes. The text should be printed in the USB terminal (pros terminal)
2. I want the format to be labled, but please note that the label of each value should be really clear. You can still use abreviation, such as err, out, p_term, i_term, d_term, but they should clearly represent the values. 
3. The question you brought up here can be discussed. Since printing the debug texts every 10ms can slows down the loop and the terminal also cannot really print texts that fast, I think it would be better if we have a parameter named "refreshTime" that allows the debug function to know when to print out those texts for a period of time. 