from langchain.prompts import ChatPromptTemplate
from langchain_chroma import Chroma
from langchain_ollama import OllamaLLM as Ollama
from get_embedding_function import get_embedding_function

CHROMA_PATH = "chroma"

PROMPT_TEMPLATE = """
Using the following ROS 2 documentation context, generate a single valid ROS 2 command that accomplishes the described task.
Return ONLY the command without any explanations, newlines, or formatting tags.

Examples:
- ros2 topic list
- ros2 run turtlesim turtlesim_node
- ros2 launch my_package my_launch_file.launch.py

{context}

---

Task: {question}
Generate only a valid ROS 2 command that: {prompt}.
"""


def query_rag(query_text: str) -> str:
    # Setup Chroma DB
    embedding_function = get_embedding_function()
    db = Chroma(persist_directory=CHROMA_PATH, embedding_function=embedding_function)

    # Search for relevant documents
    results = db.similarity_search_with_score(query_text, k=5)
    context_text = "\n\n---\n\n".join([doc.page_content for doc, _ in results])

    # Prepare prompt
    prompt_template = ChatPromptTemplate.from_template(PROMPT_TEMPLATE)
    prompt = prompt_template.format(context=context_text, question=query_text, prompt=query_text)
    

    # Return the prompt (NOT the LLM's response here)
    return prompt


# # Optional CLI for testing
# if __name__ == "__main__":
#     import argparse
#     parser = argparse.ArgumentParser()
#     parser.add_argument("query_text", type=str, help="The query text.")
#     args = parser.parse_args()
#     final_prompt = query_rag(args.query_text)

#     # Optional: run Ollama here to see the result
#     model = Ollama(model="llama3")
#     print("Prompt:\n", final_prompt)
#     response_text = model.invoke(final_prompt)
#     print("Response:\n", response_text)
