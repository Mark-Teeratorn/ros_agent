from langchain.prompts import ChatPromptTemplate
from langchain_chroma import Chroma
from langchain_ollama import OllamaLLM as Ollama
from langchain_ollama import OllamaEmbeddings

CHROMA_PATH = "chroma"

PROMPT_TEMPLATE = """
Using the following ROS 2 documentation context, generate a single valid ROS 2 command that accomplishes the described task.
Return ONLY the command without any explanations, newlines, or formatting tags.

Examples:
- ros2 topic list
- ros2 run turtlesim turtlesim_node
- ros2 launch my_package my_launch_file.launch.py

Example commands for the ired robot:
"Move /ired forward."
Output: ros2 topic pub /ired/motor/speed_control ired_msgs/msg/MotorSpeed "{{speed: [-80, 80.0, 0.0, 0.0]}}" --rate 5

"Move /ired forward for 2 second."
Output: ros2 topic pub -t 2 /ired/motor/speed_control ired_msgs/msg/MotorSpeed "{{speed: [-80, 80.0, 0.0, 0.0]}}" --rate 5

Request: "Spin /ired clockwise."
Output: ros2 topic pub /ired/motor/speed_control ired_msgs/msg/MotorSpeed "{{speed: [-20, -20.0, 0.0, 0.0]}}" --rate 5

Request: "Spin /ired counterclockwise."
Output: ros2 topic pub /ired/motor/speed_control ired_msgs/msg/MotorSpeed "{{speed: [20, 20.0, 0.0, 0.0]}}" --rate 5

Request: "Turn /ired left around 90 degree."
Output: ros2 topic pub -t 2 /ired/motor/speed_control ired_msgs/msg/MotorSpeed "{{speed: [-10, 160, 0.0, 0.0]}}" --rate 5

Request: "Turn /ired right around 90 degree."
Output: ros2 topic pub -t 2 /ired/motor/speed_control ired_msgs/msg/MotorSpeed "{{speed: [-160, 10.0, 0.0, 0.0]}}" --rate 5

Request: "Move /ired backward."
Output: ros2 topic pub /ired/motor/speed_control ired_msgs/msg/MotorSpeed "{{speed: [20, -20.0, 0.0, 0.0]}}" --rate 5

{context}

---

Task: {question}
Generate only a valid ROS 2 command that: {prompt}.
"""

def get_embedding_function():
    embeddings = OllamaEmbeddings(model="mxbai-embed-large")
    return embeddings

def query_rag(query_text: str) -> str:
    # Setup Chroma DB
    embedding_function = get_embedding_function()
    db = Chroma(persist_directory=CHROMA_PATH, embedding_function=embedding_function)

    # Search for relevant documents
    results = db.similarity_search_with_score(query_text, k=5)
    context_text = "\n\n---\n\n".join([doc.page_content for doc, _ in results])


    prompt_template = ChatPromptTemplate.from_template(PROMPT_TEMPLATE)
    prompt = prompt_template.format(context=context_text, question=query_text, prompt=query_text)
    
    return prompt



