import requests

def fetch_data_from_url(url):
    try:
        # HTTP GET isteği gönder
        response = requests.get(url)
        
        # İstek başarılı ise (status code 200) veriyi döndür
        if response.status_code == 200:
            return response.text  # .json() metodu da kullanılabilir JSON formatında veri alıyorsanız
        else:
            return f"İstek başarısız oldu. Status code: {response.status_code}"
    
    except requests.exceptions.RequestException as e:
        # Hata durumunu yakala ve mesajı döndür
        return f"Bir hata oluştu: {e}"

# Örnek kullanım:
url = "https://jsonplaceholder.typicode.com/todos/1"
data = fetch_data_from_url(url)
print(data)

