import queue

#Create a queue instance
message_queue = queue.Queue()

#Function to add a message to the queue
def add_message(message):
    message_queue.put(message)
    print(f"Message added: {message}")

Function to get a message from the queue
def get_message():
    if not message_queue.empty():
        message = message_queue.get()
        print(f"Message retrieved: {message}")
        return message
    else:
        print("No messages in the queue.")
        return None

Function to check if the queue is empty
def is_queue_empty():
    if message_queue.empty():
        print("The queue is empty.")
        return True
    else:
        print("The queue is not empty.")
        return False

Example usage
add_message("Hello, world!")
add_message("This is a test message.")

Check if the queue is empty
is_queue_empty()

Retrieve messages
retrieved_message = get_message()  # Retrieves the first message ("Hello, world!")
retrieved_message = get_message()  # Retrieves the second message ("This is a test message.")

Check if the queue is empty
is_queue_empty()

Try to retrieve a message from an empty queue
retrieved_message = get_message()  # No messages left in the queue

Check if the queue is empty again
is_queue_empty()