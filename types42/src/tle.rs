/// Unstructured TLE
/// https://en.wikipedia.org/wiki/Two-line_element_set
#[derive(Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct UnstructuredTle {
    pub satellite_name: String,
    pub line1: String,
    pub line2: String,
}
