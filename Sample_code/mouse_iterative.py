from pynput import mouse

# The event listener will be running in this block
with mouse.Events() as events:
    for event in events:
        if event.button == mouse.Button.right:
            break
        else:
            print('Received event {}'.format(event))
