# from langchain_community.embeddings.ollama import OllamaEmbeddings
# from langchain_community.embeddings.bedrock import BedrockEmbeddings


# def get_embedding_function():
#     embeddings = OllamaEmbeddings(
#         #credentials_profile_name="default", region_name="us-east-1"
#         embeddings = OllamaEmbeddings(model="nomic-embed-text")
#     )
#     # embeddings = OllamaEmbeddings(model="nomic-embed-text")
#     return embeddings

from langchain_ollama import OllamaEmbeddings

def get_embedding_function():
    embeddings = OllamaEmbeddings(model="mxbai-embed-large")
    return embeddings
