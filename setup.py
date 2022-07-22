from setuptools import setup, find_namespace_packages

with open("requirements.txt", "r") as f:
    requirements = [package.replace("\n", "") for package in f.readlines()]

setup(
    name="SearchProblemsAI",
    url="https://github.com/tboulet/AI-Agents-for-Search-Problems",
    author="Timothé Boulet",
    author_email="timothe.boulet0@gmail.com",
    
    packages=find_namespace_packages(),
    # Needed for dependencies
    install_requires=requirements[1:],
    dependency_links=requirements[:1],
        # package_data={"configs": "*.yaml"},
    version="1.0.1",
    license="MIT",
    description="SearchProblemsAI is a library of AI agents for Search Problems.",
    long_description=open('README.md').read(),
)