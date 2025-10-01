package common

type Queue []interface{} // Define a Queue type as a slice of empty interfaces

// Enqueue adds an element to the back of the queue
func (q *Queue) Enqueue(item interface{}) {
	*q = append(*q, item)
}

// Dequeue removes and returns the front element of the queue
func (q *Queue) Dequeue() interface{} {
	if q.IsEmpty() {
		return nil // Or handle error appropriately
	}
	item := (*q)[0]
	*q = (*q)[1:] // Remove the first element
	return item
}

// IsEmpty checks if the queue is empty
func (q *Queue) IsEmpty() bool {
	return len(*q) == 0
}

// Front returns the front element without removing it
func (q *Queue) Front() interface{} {
	if q.IsEmpty() {
		return nil
	}
	return (*q)[0]
}