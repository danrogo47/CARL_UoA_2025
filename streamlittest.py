import streamlit as st
import time
import random

# Title of the dashboard
st.title("Test Streamlit Dashboard")

# Text elements
st.header("Real-Time Random Number Updates")
st.write("This is a test dashboard to display random numbers and progress bars.")

# Create a placeholder for the random number and progress bars
random_number_placeholder = st.empty()
progress_bar = st.progress(0)

# Simulate a real-time update loop
for i in range(101):
    # Generate a random number between 0 and 100
    random_number = random.randint(0, 100)
    
    # Display the random number
    random_number_placeholder.write(f"Random Number: {random_number}")
    
    # Update the progress bar
    progress_bar.progress(i)
    
    # Wait for a short time before updating
    time.sleep(0.1)
    
st.write("Test completed!")
