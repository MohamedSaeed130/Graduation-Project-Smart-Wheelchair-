_access_token = None  # Private variable to store the access token

def set_access_token(token):
    """Set the access token if valid."""
    global _access_token
    if isinstance(token, str) and token.strip():
        _access_token = token.strip()
        return True
    else:
        print("Error: Invalid token. Must be a non-empty string.")
        return False

def get_access_token():
    """Get the current access token."""
    return _access_token

# Example initialization (replace with actual token or external source)
# set_access_token("your_stored_access_token_here")  # Replace with actual token