
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    action_sequence = gvm.get_variable("action_sequence", True)
    if len(action_sequence) == 0:
        return "success"
    else:
        action = action_sequence.popleft()
        print("action: ", action)
        if action == 'left':
            return "left"
        elif action == 'forward':
            return  "forward"
        elif action == 'right':
            return "right"
