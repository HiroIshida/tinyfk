current=$(strings $(ldconfig -p |grep libstd|awk '{print $4}')|grep GLIBCXX|sort --version-sort | tail -2| head -1| sed "s/^.*GLIBCXX_\([0-9.]*\).*/\1/")
maxaddmissible="3.4.21" # default in ubuntu 16.04
echo "Current version is $current"
echo "Maximum admissible version is $maxaddmissible"
if [ "$(printf '%s\n' "$maxaddmissible" "$current" | sort -V --reverse| head -n1)" = "$maxaddmissible" ]; then 
   echo "Less than or equal to ${maxaddmissible}"
else
   echo "Error: Greater than ${maxaddmissible}. Requiring older version."
   exit 1
fi
