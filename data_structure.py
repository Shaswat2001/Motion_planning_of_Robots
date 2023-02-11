from Nodes import check_nodes

class PriorityQueue:
    """
    This class is Priority Queue.
    Look into python heapq for a similar usecase
    """

    def __init__(self):
        self.queue = []

    def insert_pq(self, cost, data_node):
        """
        Insert into the Priority queue
        :param      cost:       The cost
        :type       cost:       { type_description }
        :param      data_node:  The data node
        :type       data_node:  { type_description }
        """
        node_cost_combo = (cost, data_node)
        self.queue.append(node_cost_combo)

    def pop_pq(self):
        """
        Pops the element as per the rules of priority queue
        """
        try:
            max_idx = 0
            for i in range(len(self.queue)):
                if self.queue[i][0] < self.queue[max_idx][0]:
                    max_idx = i
            max_cost, max_cost_node = self.queue[max_idx]
            del self.queue[max_idx]
            return max_cost, max_cost_node
        except IndexError:
            exit()

    def len_pq(self):
        """
        Returns the length of the priority queue
        """
        #print('Length of queue : >> ', len(self.queue))
        return len(self.queue)

    def checkinPQ(self, node_to_check):
        """
        Checks for a node in the Priority Queue
        :param node_to_check: The node to check
        :type node_to_check: Node type
        :return: True or False
        :rtype: boolean
        """
        for all_tuples in self.queue:
            if check_nodes(all_tuples[1], node_to_check):
                return True
        return False
